/*
 * Copyright (C) 2010-2012 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/fsl_devices.h>
#include <linux/gpio.h>
#include <linux/mxc_asrc.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/soc-dai.h>
#include <sound/pcm_params.h>

#include <mach/hardware.h>
#include <mach/clock.h>
#include <asm/mach-types.h>

#include "imx-esai.h"
#include "../codecs/cs42888.h"
#include "imx-pcm.h"

#if defined(CONFIG_MXC_ASRC) || defined(CONFIG_IMX_HAVE_PLATFORM_IMX_ASRC)
struct asrc_esai {
	unsigned int cpu_dai_rates;
	unsigned int codec_dai_rates;
	enum asrc_pair_index asrc_index;
	unsigned int output_sample_rate;
};
static struct asrc_esai asrc_esai_data;
static bool asrc_support = 1;

static int get_format_width(struct snd_pcm_hw_params *params)
{
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S8:
	case SNDRV_PCM_FORMAT_U8:
		return 8;
	case SNDRV_PCM_FORMAT_U16:
	case SNDRV_PCM_FORMAT_S16_LE:
	case SNDRV_PCM_FORMAT_S16_BE:
		return 16;
	case SNDRV_PCM_FORMAT_S20_3LE:
	case SNDRV_PCM_FORMAT_S20_3BE:
	case SNDRV_PCM_FORMAT_S24_3LE:
	case SNDRV_PCM_FORMAT_S24_3BE:
	case SNDRV_PCM_FORMAT_S24_BE:
	case SNDRV_PCM_FORMAT_S24_LE:
	case SNDRV_PCM_FORMAT_U24_BE:
	case SNDRV_PCM_FORMAT_U24_LE:
	case SNDRV_PCM_FORMAT_U24_3BE:
	case SNDRV_PCM_FORMAT_U24_3LE:
		return 24;
	case SNDRV_PCM_FORMAT_S32:
	case SNDRV_PCM_FORMAT_U32:
		return 32;
	default:
		pr_err("Format is not support!\r\n");
		return -EINVAL;
	}
}

static int config_asrc(struct snd_pcm_substream *substream,
					 struct snd_pcm_hw_params *params)
{
	unsigned int rate = params_rate(params);
	unsigned int channel = params_channels(params);
	unsigned int wordwidth = get_format_width(params);
	struct imx_pcm_runtime_data *pcm_data =
				substream->runtime->private_data;
	struct asrc_config config = {0};
	int ret = 0;

	if (rate <= 32000 || rate == asrc_esai_data.output_sample_rate)
		return -EINVAL;

	if (channel != 2)
		return -EINVAL;

	if (wordwidth != 24)
		return -EINVAL;

	ret = asrc_req_pair(channel, &asrc_esai_data.asrc_index);
	if (ret < 0) {
		pr_err("Fail to request asrc pair\n");
		asrc_release_pair(asrc_esai_data.asrc_index);
		asrc_finish_conv(asrc_esai_data.asrc_index);
		return -EINVAL;
	}

	config.pair = asrc_esai_data.asrc_index;
	config.channel_num = channel;
	config.input_sample_rate = rate;
	config.output_sample_rate = asrc_esai_data.output_sample_rate;
	config.inclk = OUTCLK_ASRCK1_CLK;
	config.word_width = wordwidth;
	config.outclk = OUTCLK_ESAI_TX;

	ret = asrc_config_pair(&config);
	if (ret < 0) {
		pr_err("Fail to config asrc\n");
		asrc_release_pair(asrc_esai_data.asrc_index);
		asrc_finish_conv(asrc_esai_data.asrc_index);
		return ret;
	}
	pcm_data->asrc_index = asrc_esai_data.asrc_index;
	pcm_data->asrc_enable = 1;

	return 0;
}
#else
static bool asrc_support;
#endif

struct imx_priv_state {
	int hw;
};

static struct imx_priv_state hw_state;
unsigned int mclk_freq;


static int imx_3stack_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;

	if (!cpu_dai->active) {
		hw_state.hw = 0;
	}

	if (asrc_support) {
		struct snd_soc_dai *codec_dai = rtd->codec_dai;
		asrc_esai_data.cpu_dai_rates =
			cpu_dai->driver->playback.rates;
		asrc_esai_data.codec_dai_rates =
			codec_dai->driver->playback.rates;
	}

	return 0;
}

static void imx_3stack_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;

	if (asrc_support) {
		struct snd_soc_dai *codec_dai = rtd->codec_dai;
		struct imx_pcm_runtime_data *pcm_data =
				substream->runtime->private_data;
		if (pcm_data->asrc_enable) {
			asrc_release_pair(asrc_esai_data.asrc_index);
			asrc_finish_conv(asrc_esai_data.asrc_index);
		}
		pcm_data->asrc_enable = 0;
		asrc_esai_data.asrc_index = -1;

		codec_dai->driver->playback.rates =
				asrc_esai_data.codec_dai_rates;
		cpu_dai->driver->playback.rates =
				asrc_esai_data.cpu_dai_rates;
	}

	if (!cpu_dai->active)
		hw_state.hw = 0;
}

static int imx_3stack_surround_hw_params(struct snd_pcm_substream *substream,
					 struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	unsigned int rate = params_rate(params);
	u32 dai_format;
	unsigned int lrclk_ratio = 0;

	if (hw_state.hw)
		return 0;
	hw_state.hw = 1;

	if (asrc_support &&
		(substream->stream == SNDRV_PCM_STREAM_PLAYBACK) &&
		!config_asrc(substream, params)) {
		rate = asrc_esai_data.output_sample_rate;
	}
	if (cpu_is_mx53() || machine_is_mx6q_sabreauto()) {
		switch (rate) {
		case 32000:
			lrclk_ratio = 3;
			break;
		case 48000:
			lrclk_ratio = 3;
			break;
		case 64000:
			lrclk_ratio = 1;
			break;
		case 96000:
			lrclk_ratio = 1;
			break;
		case 128000:
			lrclk_ratio = 1;
			break;
		case 44100:
			lrclk_ratio = 3;
			break;
		case 88200:
			lrclk_ratio = 1;
			break;
		case 176400:
			lrclk_ratio = 0;
			break;
		case 192000:
			lrclk_ratio = 0;
			break;
		default:
			pr_info("Rate not support.\n");
			return -EINVAL;;
		}
	} else if (cpu_is_mx6q() || cpu_is_mx6dl()) {
		switch (rate) {
		case 32000:
			lrclk_ratio = 5;
			break;
		case 48000:
			lrclk_ratio = 5;
			break;
		case 64000:
			lrclk_ratio = 2;
			break;
		case 96000:
			lrclk_ratio = 2;
			break;
		case 128000:
			lrclk_ratio = 2;
			break;
		case 44100:
			lrclk_ratio = 5;
			break;
		case 88200:
			lrclk_ratio = 2;
			break;
		case 176400:
			lrclk_ratio = 0;
			break;
		case 192000:
			lrclk_ratio = 0;
			break;
		default:
			pr_info("Rate not support.\n");
			return -EINVAL;;
		}
	}
	dai_format = SND_SOC_DAIFMT_LEFT_J | SND_SOC_DAIFMT_NB_NF |
	    SND_SOC_DAIFMT_CBS_CFS;

	/* set cpu DAI configuration */
	snd_soc_dai_set_fmt(cpu_dai, dai_format);
	/* set i.MX active slot mask */
	snd_soc_dai_set_tdm_slot(cpu_dai, 0x3, 0x3, 2, 32);
	/* set the ESAI system clock as output */
	if (cpu_is_mx53() || machine_is_mx6q_sabreauto()) {
		snd_soc_dai_set_sysclk(cpu_dai, ESAI_CLK_EXTAL,
			mclk_freq, SND_SOC_CLOCK_OUT);
	} else if (cpu_is_mx6q() || cpu_is_mx6dl()) {
		snd_soc_dai_set_sysclk(cpu_dai, ESAI_CLK_EXTAL_DIV,
			mclk_freq, SND_SOC_CLOCK_OUT);
	}
	/* set the ratio */
	snd_soc_dai_set_clkdiv(cpu_dai, ESAI_TX_DIV_PSR, 1);
	if (cpu_is_mx53() || machine_is_mx6q_sabreauto())
		snd_soc_dai_set_clkdiv(cpu_dai, ESAI_TX_DIV_PM, 0);
	else if (cpu_is_mx6q() || cpu_is_mx6dl())
		snd_soc_dai_set_clkdiv(cpu_dai, ESAI_TX_DIV_PM, 2);
	snd_soc_dai_set_clkdiv(cpu_dai, ESAI_TX_DIV_FP, lrclk_ratio);

	snd_soc_dai_set_clkdiv(cpu_dai, ESAI_RX_DIV_PSR, 1);
	if (cpu_is_mx53() || machine_is_mx6q_sabreauto())
		snd_soc_dai_set_clkdiv(cpu_dai, ESAI_RX_DIV_PM, 0);
	else if (cpu_is_mx6q() || cpu_is_mx6dl())
		snd_soc_dai_set_clkdiv(cpu_dai, ESAI_RX_DIV_PM, 2);
	snd_soc_dai_set_clkdiv(cpu_dai, ESAI_RX_DIV_FP, lrclk_ratio);

	/* set codec DAI configuration */
	snd_soc_dai_set_fmt(codec_dai, dai_format);
	/* set codec Master clock */
	snd_soc_dai_set_sysclk(codec_dai, 0, mclk_freq, SND_SOC_CLOCK_IN);

	return 0;
}

static struct snd_soc_ops imx_3stack_surround_ops = {
	.startup = imx_3stack_startup,
	.shutdown = imx_3stack_shutdown,
	.hw_params = imx_3stack_surround_hw_params,
};

static const struct snd_soc_dapm_widget imx_3stack_dapm_widgets[] = {
	SND_SOC_DAPM_LINE("Line Out Jack", NULL),
	SND_SOC_DAPM_LINE("Line In Jack", NULL),
};

static const struct snd_soc_dapm_route audio_map[] = {
	/* Line out jack */
	{"Line Out Jack", NULL, "AOUT1L"},
	{"Line Out Jack", NULL, "AOUT1R"},
	{"Line Out Jack", NULL, "AOUT2L"},
	{"Line Out Jack", NULL, "AOUT2R"},
	{"Line Out Jack", NULL, "AOUT3L"},
	{"Line Out Jack", NULL, "AOUT3R"},
	{"Line Out Jack", NULL, "AOUT4L"},
	{"Line Out Jack", NULL, "AOUT4R"},
	{"AIN1L", NULL, "Line In Jack"},
	{"AIN1R", NULL, "Line In Jack"},
	{"AIN2L", NULL, "Line In Jack"},
	{"AIN2R", NULL, "Line In Jack"},
};

static int imx_3stack_cs42888_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;

	if (asrc_support)
		asrc_esai_data.output_sample_rate = 44100;

	snd_soc_dapm_new_controls(&codec->dapm, imx_3stack_dapm_widgets,
				  ARRAY_SIZE(imx_3stack_dapm_widgets));

	snd_soc_dapm_add_routes(&codec->dapm, audio_map, ARRAY_SIZE(audio_map));

	snd_soc_dapm_sync(&codec->dapm);

	return 0;
}

static struct snd_soc_dai_link imx_3stack_dai[] = {
	{
	.name = "HiFi",
	.stream_name = "HiFi",
	.codec_dai_name = "CS42888",
#ifdef CONFIG_SOC_IMX53
	.codec_name = "cs42888.1-0048",
#endif
#ifdef CONFIG_SOC_IMX6Q
	.codec_name = "cs42888.0-0048",
#endif
	.cpu_dai_name = "imx-esai.0",
	.platform_name = "imx-pcm-audio.0",
	.init = imx_3stack_cs42888_init,
	.ops = &imx_3stack_surround_ops,
	},
};

static struct snd_soc_card snd_soc_card_imx_3stack = {
	.name = "cs42888-audio",
	.dai_link = imx_3stack_dai,
	.num_links = ARRAY_SIZE(imx_3stack_dai),
};

/*
 * This function will register the snd_soc_pcm_link drivers.
 */
static int __devinit imx_3stack_cs42888_probe(struct platform_device *pdev)
{
	struct mxc_audio_platform_data *plat_data = pdev->dev.platform_data;

	if (!plat_data) {
		dev_err(&pdev->dev, "plat_data is missing\n");
		return -EINVAL;
	}
	mclk_freq = plat_data->sysclk;
	if (plat_data->codec_name)
		imx_3stack_dai[0].codec_name = plat_data->codec_name;
	return 0;
}

static int __devexit imx_3stack_cs42888_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver imx_3stack_cs42888_driver = {
	.probe = imx_3stack_cs42888_probe,
	.remove = __devexit_p(imx_3stack_cs42888_remove),
	.driver = {
		   .name = "imx-cs42888",
		   .owner = THIS_MODULE,
		   },
};

static struct platform_device *imx_3stack_snd_device;

static int __init imx_3stack_asoc_init(void)
{
	int ret;
	pr_info("imx_3stack asoc driver\n");
	ret = platform_driver_register(&imx_3stack_cs42888_driver);
	if (ret < 0)
		goto exit;

	imx_3stack_snd_device = platform_device_alloc("soc-audio", 2);
	if (!imx_3stack_snd_device)
		goto err_device_alloc;
	platform_set_drvdata(imx_3stack_snd_device, &snd_soc_card_imx_3stack);
	ret = platform_device_add(imx_3stack_snd_device);
	if (0 == ret)
		goto exit;

	platform_device_unregister(imx_3stack_snd_device);
err_device_alloc:
	platform_driver_unregister(&imx_3stack_cs42888_driver);
exit:
	return ret;
}

static void __exit imx_3stack_asoc_exit(void)
{
	platform_driver_unregister(&imx_3stack_cs42888_driver);
	platform_device_unregister(imx_3stack_snd_device);
}

module_init(imx_3stack_asoc_init);
module_exit(imx_3stack_asoc_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("ALSA SoC cs42888 Machine Layer Driver");
MODULE_LICENSE("GPL");
