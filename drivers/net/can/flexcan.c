/**
 * @file flexcan.c
 * @brief FLEXCAN CAN controller driver
 * @details This file is driver for work with flexcan controller
 * @version 1.1.25
 * @date 04.04.2015
 * @author Mikita Dzivakou
 * @copyright (c) 2005-2006 Varma Electronics Oy
 * @copyright (c) 2009 Sascha Hauer, Pengutronix
 * @copyright (c) 2010 Marc Kleine-Budde, Pengutronix
 * @copyright (c) 2015 Mikita Dzivakou, Strim-tech
 * @verbatim
   LICENCE:
   This program is free software; you can redistribute it and/or
   modify it under the terms of the GNU General Public License as
   published by the Free Software Foundation version 2.
   
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
 * @endverbatim
 */

// #include <linux/netdevice.h>
#include <linux/can/error.h>
#include <linux/can/platform/flexcan.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/if_arp.h>
#include <linux/if_ether.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/time.h>

#include <linux/types.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/kdev_t.h>
#include <linux/version.h>
#include <asm/io.h>
#include <linux/string.h>

#include <mach/clock.h>
#include <mach/hardware.h>

#include "chr_flexcan.h"

#ifdef CONFIG_ARCH_MXC
	#include <mach/iomux-v3.h>
#endif

#define DRV_NAME		"flexcan"
#define DRV_VER			"1.2.25" 

/* 6 for RX fifo and 2 error handling */
#define FLEXCAN_NAPI_WEIGHT				(6 + 1)

/* FLEXCAN module configuration register (CANMCR) bits */
#define FLEXCAN_MCR_MDIS				BIT(31)
#define FLEXCAN_MCR_FRZ					BIT(30)
#define FLEXCAN_MCR_FEN					BIT(29)
#define FLEXCAN_MCR_HALT				BIT(28)
#define FLEXCAN_MCR_NOT_RDY				BIT(27)
#define FLEXCAN_MCR_WAK_MSK				BIT(26)
#define FLEXCAN_MCR_SOFTRST				BIT(25)
#define FLEXCAN_MCR_FRZ_ACK				BIT(24)
#define FLEXCAN_MCR_SUPV				BIT(23)
#define FLEXCAN_MCR_SLF_WAK				BIT(22)
#define FLEXCAN_MCR_WRN_EN				BIT(21)
#define FLEXCAN_MCR_LPM_ACK				BIT(20)
#define FLEXCAN_MCR_WAK_SRC				BIT(19)
#define FLEXCAN_MCR_DOZE				BIT(18)
#define FLEXCAN_MCR_SRX_DIS				BIT(17)
#define FLEXCAN_MCR_BCC					BIT(16)
#define FLEXCAN_MCR_LPRIO_EN			BIT(13)
#define FLEXCAN_MCR_AEN					BIT(12)
#define FLEXCAN_MCR_MAXMB(x)			((x) & 0xf)
#define FLEXCAN_MCR_IDAM_A				(0 << 8)
#define FLEXCAN_MCR_IDAM_B				(1 << 8)
#define FLEXCAN_MCR_IDAM_C				(2 << 8)
#define FLEXCAN_MCR_IDAM_D				(3 << 8)

/* FLEXCAN control register (CANCTRL) bits */
#define FLEXCAN_CTRL_PRESDIV(x)			(((x) & 0xff) << 24)
#define FLEXCAN_CTRL_RJW(x)				(((x) & 0x03) << 22)
#define FLEXCAN_CTRL_PSEG1(x)			(((x) & 0x07) << 19)
#define FLEXCAN_CTRL_PSEG2(x)			(((x) & 0x07) << 16)
#define FLEXCAN_CTRL_BOFF_MSK			BIT(15)
#define FLEXCAN_CTRL_ERR_MSK			BIT(14)
#define FLEXCAN_CTRL_CLK_SRC			BIT(13)
#define FLEXCAN_CTRL_LPB				BIT(12)
#define FLEXCAN_CTRL_TWRN_MSK			BIT(11)
#define FLEXCAN_CTRL_RWRN_MSK			BIT(10)
#define FLEXCAN_CTRL_SMP				BIT(7)
#define FLEXCAN_CTRL_BOFF_REC			BIT(6)
#define FLEXCAN_CTRL_TSYN				BIT(5)
#define FLEXCAN_CTRL_LBUF				BIT(4)
#define FLEXCAN_CTRL_LOM				BIT(3)
#define FLEXCAN_CTRL_PROPSEG(x)			((x) & 0x07)

#define FLEXCAN_CTRL_ERR_BUS			(FLEXCAN_CTRL_ERR_MSK)
#define FLEXCAN_CTRL_ERR_STATE 			(FLEXCAN_CTRL_TWRN_MSK | FLEXCAN_CTRL_RWRN_MSK | FLEXCAN_CTRL_BOFF_MSK)
#define FLEXCAN_CTRL_ERR_ALL 			(FLEXCAN_CTRL_ERR_BUS | FLEXCAN_CTRL_ERR_STATE)

/* FLEXCAN error and status register (ESR) bits */
#define FLEXCAN_ESR_TWRN_INT			BIT(17)
#define FLEXCAN_ESR_RWRN_INT			BIT(16)
#define FLEXCAN_ESR_BIT1_ERR			BIT(15)
#define FLEXCAN_ESR_BIT0_ERR			BIT(14)
#define FLEXCAN_ESR_ACK_ERR				BIT(13)
#define FLEXCAN_ESR_CRC_ERR				BIT(12)
#define FLEXCAN_ESR_FRM_ERR				BIT(11)
#define FLEXCAN_ESR_STF_ERR				BIT(10)
#define FLEXCAN_ESR_TX_WRN				BIT(9)
#define FLEXCAN_ESR_RX_WRN				BIT(8)
#define FLEXCAN_ESR_IDLE				BIT(7)
#define FLEXCAN_ESR_TXRX				BIT(6)
#define FLEXCAN_EST_FLT_CONF_SHIFT		(4)
#define FLEXCAN_ESR_FLT_CONF_MASK		(0x3 << FLEXCAN_EST_FLT_CONF_SHIFT)
#define FLEXCAN_ESR_FLT_CONF_ACTIVE		(0x0 << FLEXCAN_EST_FLT_CONF_SHIFT)
#define FLEXCAN_ESR_FLT_CONF_PASSIVE	(0x1 << FLEXCAN_EST_FLT_CONF_SHIFT)
#define FLEXCAN_ESR_BOFF_INT			BIT(2)
#define FLEXCAN_ESR_ERR_INT				BIT(1)
#define FLEXCAN_ESR_WAK_INT				BIT(0)

#define FLEXCAN_ESR_ERR_BUS 			(FLEXCAN_ESR_BIT1_ERR | FLEXCAN_ESR_BIT0_ERR | FLEXCAN_ESR_ACK_ERR | \
										 FLEXCAN_ESR_CRC_ERR | FLEXCAN_ESR_FRM_ERR | FLEXCAN_ESR_STF_ERR)
#define FLEXCAN_ESR_ERR_STATE 			(FLEXCAN_ESR_TWRN_INT | FLEXCAN_ESR_RWRN_INT | FLEXCAN_ESR_BOFF_INT)
#define FLEXCAN_ESR_ERR_ALL 			(FLEXCAN_ESR_ERR_BUS | FLEXCAN_ESR_ERR_STATE)

/* FLEXCAN interrupt flag register (IFLAG) bits */
#define FLEXCAN_TX_BUF_ID				8
#define FLEXCAN_IFLAG_BUF(x)			BIT(x)
#define FLEXCAN_IFLAG_RX_FIFO_OVERFLOW	BIT(7)
#define FLEXCAN_IFLAG_RX_FIFO_WARN		BIT(6)
#define FLEXCAN_IFLAG_RX_FIFO_AVAILABLE	BIT(5)

#define FLEXCAN_IFLAG_DEFAULT 			(FLEXCAN_IFLAG_RX_FIFO_OVERFLOW | FLEXCAN_IFLAG_RX_FIFO_AVAILABLE | FLEXCAN_IFLAG_BUF(FLEXCAN_TX_BUF_ID))

/* FLEXCAN message buffers */
#define FLEXCAN_MB_CNT_CODE(x)			(((x) & 0xf) << 24)
#define FLEXCAN_MB_CNT_SRR				BIT(22)
#define FLEXCAN_MB_CNT_IDE				BIT(21)
#define FLEXCAN_MB_CNT_RTR				BIT(20)
#define FLEXCAN_MB_CNT_LENGTH(x)		(((x) & 0xf) << 16)
#define FLEXCAN_MB_CNT_TIMESTAMP(x)		((x) & 0xffff)

#define FLEXCAN_MB_CODE_MASK 			(0xf0ffffff)


#define FLEXCAN_BTRT_1000_SP866		(0x01290005)
#define FLEXCAN_BTRT_1000_SP800		(0x012a0004)
#define FLEXCAN_BTRT_1000_SP733		(0x01230004)
#define FLEXCAN_BTRT_1000_SP700		(0x02120002)

#define FLEXCAN_BTRT_1000			(0x01290005)
#define FLEXCAN_BTRT_500			(0x023b0006)		/* Предварительно, настройка рассчитана через canconfig */
#define FLEXCAN_BTRT_250			(0x053b0006)		/* Предварительно, настройка рассчитана через canconfig */
#define FLEXCAN_BTRT_125			(0x0b3b0006)		/* Предварительно, настройка рассчитана через canconfig */
#define FLEXCAN_BTRT_100			(0x0e3b0006)		/* Предварительно, настройка рассчитана через canconfig */
#define FLEXCAN_BTRT_DFLT			FLEXCAN_BTRT_1000

#define FLEXCAN_BTRT_MASK			(0xFFFF0007)

#define DEV_FIRST  					(0) 
#define DEV_COUNT  					(2) 

#define BUF_RECV_CAP				(65536)
#define BUF_SEND_CAP				(64)

#define BUF_RECV_MASK				(fimx6d.f_dev[dev_num].buf_recv_size - 1)
#define BUF_SEND_MASK				(fimx6d.f_dev[dev_num].buf_send_size - 1)

#define TYPE_BUF_RECV				(0x0F)
#define TYPE_BUF_SEND				(0xF0)

#define IRQ_BASE					(142)

/* Structure of the hardware registers */
struct flexcan_regs {
	u32 mcr;		/* 0x00 */
	u32 ctrl;		/* 0x04 */
	u32 timer;		/* 0x08 */
	u32 _reserved1;		/* 0x0c */
	u32 rxgmask;		/* 0x10 */
	u32 rx14mask;		/* 0x14 */
	u32 rx15mask;		/* 0x18 */
	u32 ecr;		/* 0x1c */
	u32 esr;		/* 0x20 */
	u32 imask2;		/* 0x24 */
	u32 imask1;		/* 0x28 */
	u32 iflag2;		/* 0x2c */
	u32 iflag1;		/* 0x30 */
	u32 crl2;		/* 0x34 */
	u32 esr2;		/* 0x38 */
	u32 _reserved2[2];
	u32 crcr;		/* 0x44 */
	u32 rxfgmask;		/* 0x48 */
	u32 rxfir;		/* 0x4c */
	u32 _reserved3[12];
	struct flexcan_mb cantxfg[64];
};

enum flexcan_ip_version {
	FLEXCAN_VER_3_0_0,
	FLEXCAN_VER_3_0_4,
	FLEXCAN_VER_10_0_12,
};

static struct can_bittiming_const flexcan_bittiming_const = {
	.name = DRV_NAME,
	.tseg1_min = 4,
	.tseg1_max = 16,
	.tseg2_min = 2,
	.tseg2_max = 8,
	.sjw_max = 4,
	.brp_min = 1,
	.brp_max = 256,
	.brp_inc = 1,
};

typedef struct {
	unsigned int n_first, n_last;
} circle_buf_t;

struct flexcan_frame_mb {
	struct flexcan_mb mb;
	struct timeval time;
};

struct flexcan_chardev {
/*
 * Имя устройства.
 * Это же имя отдается файлу устройства в /dev и /proc
 */
	char name[IFNAMSIZ];
/*
 * Содержит major и minor номер зарегистрированного устройства
 * Номера извлекаются макросами MAJOR(dev_t devt) и MINOR(dev_t devt)
 * Номер формируется при регистрации устройства автоматически
 * Так же формируется макросом MKDEV(major, minor)
 */
	dev_t f_devt;				// major и minor номера устройства
/*
 * Адрес начала блока памяти устройства
 * Сохраняется при регистрации устройства
 * Используется для чтения регистров устройства с помощью struct flexcan_regs
 */
	void __iomem *f_base;
/*
 * Номер вектора прерывания для устройства
 */
	u8 irq_num;					// IRQ номер

/*
 * Флаг открытия файла устройства
 * Нужен для предотвращения множественного доступа к файлу
 * (а еще для запрета удаления драйвера при открытом файле, но не реализовано)
 */
	u8 is_open;					// флаг открытия файла
/*
 * Значение регистра CTRL для настройки скорости передачи и режима работы
 * При инициализации устанавливается значение по умолчанию FLEXCAN_BTRT_DFLT
 * Изменяется через IOCTL запрос
 */
	u32 reg_ctrl_bittiming;
/*
 * Значение по умолчанию регистра CTRL для будущих использований
 * Создается в функции chip_start
 */
	u32 reg_ctrl_default;

/*
 * Структура символьного утсройства.
 */
	struct cdev f_cdev;	
/*
 * Указатель на структуру встроенного в контроллер устройства flexcan
 */			
	struct flexcan_platform_data *pdata;
/*
 * Структура с данными файла устройства в файловой системе /proc 
 */
	struct proc_dir_entry *dev_proc_file;

/*
 * Структура с данными по тактированию устройства
 */
	struct clk *clk;
/*
 * Структура с данными о настройках таймингов устройства
 */
	struct can_bittiming bittiming;
/*
 * Структура с данными о состоянии и работе устройства
 * Содержит различные счетчики и переменные состояния устройства
 */
	struct flexcan_stats stats;

/*
 * Размер буфера для сохранения принятых сообщений
 */
	u32 buf_recv_size;
/*
 * Номера ячеек для записи и чтения циклического буффера для сохранения
 */
	circle_buf_t frame_buf_recv;
/*
 * Циклический буффер для сохранения принятых сообщений
 */
	struct flexcan_frame_mb buf_recv[BUF_RECV_CAP];

/*
 * Размер буфера для сохранения сообщений на отправку
 */
	u32 buf_send_size;
/*
 * Номера ячеек для записи и чтения циклического буффера сообщений на отправку
 */
	circle_buf_t frame_buf_send;
/*
 * Циклический буффер для сохранения сообщений на отправку
 */
	struct can_frame buf_send[BUF_SEND_CAP];

	// int (*do_set_bittiming)(__u8 *dev_num, struct can_bittiming *bittiming);
	int (*do_set_mode)(__u8 *dev_num, enum can_mode mode);					// вернуть когда буду убирать netdevice
	// int (*do_get_state)(__u8 *dev_num, enum can_state *state);
	int (*do_get_berr_counter)(__u8 *dev_num, struct flexcan_stats *bec);
};

struct flexcan_device	{
	char name[PLATFORM_NAME_SIZE];						
	u8 is_init;					// флаг инициализации
	dev_t f_devt;				// major номер группы

	enum flexcan_ip_version version;

	struct proc_dir_entry *dev_proc_dir;
	struct class *f_class;	
	struct flexcan_chardev f_dev[DEV_COUNT];
};

static struct flexcan_device fimx6d;

//static void pp(char *p,...){}
//static void (*my_debug)(char *p,...) = pp;
// static void (*// 1my_debug)(char *p,...) = printk;

static unsigned int flexcan_start_transmit(u8 dev_num);
static void flexcan_set_bittiming(const u8 dev_num, const u32 reg_ctrl);
static void flexcan_chip_stop(const u8 dev_num);
static int flexcan_chip_start(const u8 dev_num);

//-----------------------------------------------------------------------------------------------------------------------

// ######################################################################################################################
// ##############################	Чисто временная мера, потом вернуть код на место!!!	  ###############################
// ######################################################################################################################
// #include "code_buf.c"
static inline unsigned int flexcan_recv_buf_is_empty(u8 dev_num)
{
	return (fimx6d.f_dev[dev_num].frame_buf_recv.n_first == fimx6d.f_dev[dev_num].frame_buf_recv.n_last);
}
static inline unsigned int flexcan_send_buf_is_empty(u8 dev_num)
{
	return (fimx6d.f_dev[dev_num].frame_buf_send.n_first == fimx6d.f_dev[dev_num].frame_buf_send.n_last);
}

static inline unsigned int flexcan_recv_buf_is_full(u8 dev_num)
{
	unsigned int n_next_first = (fimx6d.f_dev[dev_num].frame_buf_recv.n_first + 1) & BUF_RECV_MASK;
	return (n_next_first == fimx6d.f_dev[dev_num].frame_buf_recv.n_last);
}
static inline unsigned int flexcan_send_buf_is_full(u8 dev_num)
{
	unsigned int n_next_first = (fimx6d.f_dev[dev_num].frame_buf_send.n_first + 1) & BUF_SEND_MASK;
	return (n_next_first == fimx6d.f_dev[dev_num].frame_buf_send.n_last);
}

static inline unsigned int flexcan_recv_buf_push(u8 dev_num, const struct flexcan_mb *s_mb, const struct timeval *s_tv)
{
	if(flexcan_recv_buf_is_full(dev_num)) 	{
		return 1;
	}

	struct flexcan_frame_mb *buf = (struct flexcan_frame_mb *) (fimx6d.f_dev[dev_num].buf_recv + fimx6d.f_dev[dev_num].frame_buf_recv.n_first);
	memcpy(&buf->mb, s_mb, sizeof(struct flexcan_mb));
	memcpy(&buf->time, s_tv, sizeof(struct timeval));

	fimx6d.f_dev[dev_num].frame_buf_recv.n_first = (fimx6d.f_dev[dev_num].frame_buf_recv.n_first + 1) & BUF_RECV_MASK;

	return 0;
}
static inline unsigned int flexcan_send_buf_push(u8 dev_num, const struct can_frame *s_cf)
{
	circle_buf_t *f_buf = &fimx6d.f_dev[dev_num].frame_buf_send;
	unsigned int n_next_first = (f_buf->n_first + 1) & BUF_SEND_MASK;

	unsigned int buf_ret = flexcan_send_buf_is_full(dev_num);
	if(buf_ret) 	{
		return 1;
	}
	else 	{
		struct can_frame *buf = (struct can_frame *) (fimx6d.f_dev[dev_num].buf_send + f_buf->n_first);
		memcpy(buf, s_cf, sizeof(struct can_frame));

		f_buf->n_first = n_next_first;
	}

	return 0;
}

static inline unsigned int flexcan_recv_buf_pop(u8 dev_num, struct flexcan_frame_mb *s_frame)
{
	circle_buf_t *f_buf = &fimx6d.f_dev[dev_num].frame_buf_recv;

	unsigned int buf_ret = flexcan_recv_buf_is_empty(dev_num);
	if(buf_ret)	{
		return 1;
	}
	else 	{
		*(struct flexcan_frame_mb *) s_frame = *(struct flexcan_frame_mb *) (fimx6d.f_dev[dev_num].buf_recv + f_buf->n_last);
		f_buf->n_last = (f_buf->n_last + 1) & BUF_RECV_MASK;
	}
	
	return 0;
}
static inline unsigned int flexcan_send_buf_pop(u8 dev_num, struct can_frame *s_frame)
{
	circle_buf_t *f_buf = &fimx6d.f_dev[dev_num].frame_buf_send;

	unsigned int buf_ret = flexcan_send_buf_is_empty(dev_num);
	if(buf_ret)	{
		return 1;
	}
	else 	{
		*(struct can_frame *) s_frame = *(struct can_frame *) (fimx6d.f_dev[dev_num].buf_send + f_buf->n_last);
		f_buf->n_last = (f_buf->n_last + 1) & BUF_SEND_MASK;
	}

	return 0;
}

static inline unsigned int flexcan_recv_buf_free_space(u8 dev_num)
{
	circle_buf_t *f_buf = &fimx6d.f_dev[dev_num].frame_buf_recv;
	unsigned int buf_free_space = (f_buf->n_last - f_buf->n_first - 1) & BUF_RECV_MASK;
	return buf_free_space;
}
static inline unsigned int flexcan_send_buf_free_space(u8 dev_num)
{
	circle_buf_t *f_buf = &fimx6d.f_dev[dev_num].frame_buf_send;
	unsigned int buf_free_space = (f_buf->n_last - f_buf->n_first - 1) & BUF_SEND_MASK;
	return buf_free_space;
}

static inline unsigned int flexcan_recv_buf_data_size(u8 dev_num)
{
	circle_buf_t *f_buf = &fimx6d.f_dev[dev_num].frame_buf_recv;
	unsigned int buf_data_size = (f_buf->n_first - f_buf->n_last) & BUF_RECV_MASK;
	return buf_data_size;
}
static inline unsigned int flexcan_send_buf_data_size(u8 dev_num)
{
	circle_buf_t *f_buf = &fimx6d.f_dev[dev_num].frame_buf_send;
	unsigned int buf_data_size = (f_buf->n_first - f_buf->n_last) & BUF_SEND_MASK;
	return buf_data_size;
}
// ######################################################################################################################
// ######################################################################################################################



// ######################################################################################################################
// ##############################	Чисто временная мера, потом вернуть код на место!!!	  ###############################
// ######################################################################################################################
// #include "code_char.c"
static ssize_t flexcan_char_write(struct file *file, const char __user *buf, size_t length, loff_t *off)
{
	struct inode *inode = file->f_path.dentry->d_inode;
	u8 dev_num = iminor(inode);

	struct flexcan_frame_cf frame;
	struct flexcan_frame_cf *user_buf_ptr = (struct flexcan_frame_cf *) buf;

	unsigned int i = 0, cp_ret = 0, buf_ret = 0;
	unsigned int frame_need_read = 0;

	// 1my_debug("%s.%d: %s start work\n", fimx6d.name, dev_num, __func__);

	/* Проверяем размер сообщения */
	if(length < sizeof(struct flexcan_frame_cf))	{
		// 1my_debug("%s.%d: %s return 0\n", fimx6d.name, dev_num, __func__);
		return 0;
	}
	frame_need_read = (((int) length) / sizeof(struct flexcan_frame_cf));

	// 1my_debug("%s.%d: %s need read %d frames from user\n", fimx6d.name, dev_num, __func__, frame_need_read);
	/* Проверяем полный ли буффер */
	buf_ret = flexcan_send_buf_is_full(dev_num);
	if(buf_ret)	{
		// 1my_debug("%s.%d: %s bufer is full, return 0\n", fimx6d.name, dev_num, __func__);
		return 0;
	}

	/* Получение данных */
	// 1my_debug("%s.%d: %s start read %d frames\n", fimx6d.name, dev_num, __func__, frame_need_read);

	for(i = 0; i < frame_need_read; i++)	{
		cp_ret = copy_from_user(&frame, user_buf_ptr++, sizeof(struct flexcan_frame_cf));
		if(cp_ret)	{
			// 1my_debug("%s.%d: %s copy_from_user can't copy = %d bytes\n", fimx6d.name, (int) dev_num, __func__, cp_ret);
		}
		else 	{
			buf_ret = flexcan_send_buf_push(dev_num, &frame.cf);
			if (buf_ret) {
				break;
			}
		}
	}
	flexcan_start_transmit(dev_num);

    return (i * sizeof(struct flexcan_frame_cf));
}


static void flexcan_decode_frame(struct can_frame *cf, const struct flexcan_mb *mb)
{
	if (mb->can_ctrl & FLEXCAN_MB_CNT_IDE)	{
		cf->can_id = ((mb->can_id >> 0) & CAN_EFF_MASK) | CAN_EFF_FLAG;
	}
	else 	{
		cf->can_id = (mb->can_id >> 18) & CAN_SFF_MASK;
	}

	if (mb->can_ctrl & FLEXCAN_MB_CNT_RTR)	{
		cf->can_id |= CAN_RTR_FLAG;
	}
	cf->can_dlc = ((mb->can_ctrl >> 16) & 0xf);

	if(cf->can_dlc == 0)	{
		*(__be32 *)(cf->data + 0) = 0x00000000;
		*(__be32 *)(cf->data + 4) = 0x00000000;
	}
	else if(cf->can_dlc <= 4)	{
		*(__be32 *)(cf->data + 0) = cpu_to_be32(readl(&mb->data[0]));
		*(__be32 *)(cf->data + 4) = 0x00000000;		
	}
	else 	{
		*(__be32 *)(cf->data + 0) = cpu_to_be32(readl(&mb->data[0]));
		*(__be32 *)(cf->data + 4) = cpu_to_be32(readl(&mb->data[1]));
	}
}


static ssize_t flexcan_char_read(struct file *file, char __user *buf, size_t length_read, loff_t *off)
{
	struct inode *inode = file->f_path.dentry->d_inode;
	u8 dev_num = iminor(inode);

	struct flexcan_frame_mb read_frame;
	struct flexcan_frame_cf send_frame;
	struct flexcan_frame_cf *user_buf_ptr = (struct flexcan_frame_cf *) buf;

	size_t length = length_read;
	unsigned int i = 0, cp_ret = 0, buf_ret = 0;
	unsigned int frame_need_send = 0;

	/* Проверяем размер буффера пользователя */
	if(length < sizeof(struct flexcan_frame_cf))	{
		// 1my_debug("%s.%d: %s return 0\n", fimx6d.name, dev_num, __func__);
		return 0;
	}
	frame_need_send = (((int) length) / sizeof(struct flexcan_frame_cf));

	/* Проверяем пустой ли буффер */
	buf_ret = flexcan_recv_buf_is_empty(dev_num);
	if(buf_ret)	{
		return 0;
	}

	/* Отправка данных */
	// 1my_debug("%s.%d: %s start send %d frames\n", fimx6d.name, dev_num, __func__, frame_need_send);
	for(i = 0; i < frame_need_send; i++)	{
		buf_ret = flexcan_recv_buf_pop(dev_num, &read_frame);
		if (buf_ret) {
			break;
		}
		else 	{
			memcpy(&send_frame.time, &read_frame.time, sizeof(struct timeval));
			flexcan_decode_frame(&send_frame.cf, &read_frame.mb);

			// 1my_debug("%s.%d: %s copy_to_user %#08x.%#08x %#08x %#02x [0x%02x%02x%02x%02x,0x%02x%02x%02x%02x]\n", fimx6d.name, dev_num, __func__, send_frame.time.tv_sec, send_frame.time.tv_usec, send_frame.cf.can_id, send_frame.cf.can_dlc, send_frame.cf.data[0], send_frame.cf.data[1], send_frame.cf.data[2], send_frame.cf.data[3], send_frame.cf.data[4], send_frame.cf.data[5], send_frame.cf.data[6], send_frame.cf.data[7]);
			cp_ret = copy_to_user(user_buf_ptr++, &send_frame, sizeof(struct flexcan_frame_cf));	
			if(cp_ret)	{
				// 1my_debug("%s.%d: %s copy_to_user can't copy = %d bytes\n", fimx6d.name, (int) dev_num, __func__, cp_ret);
				break;
			}
			else 	{
				// 1my_debug("%s.%d: %s user_buf_ptr = %#08x\n", fimx6d.name, (int) dev_num, __func__, (int) user_buf_ptr);
			}
		}
	}

	return (i * sizeof(struct flexcan_frame_cf));
}

//IOCTL Func ------------------------------------------------------------------------------------------------------------------------------------
static int flexcan_char_ioctl(struct file *file, unsigned int ioctl_num, unsigned long ioctl_param)
{	
	struct inode *inode = file->f_path.dentry->d_inode;
	const u8 dev_num = iminor(inode);
	static u32 read_settings = 0;
	u32 settings_debug = 0, settings_mode = 0, settings_bitrate = 0;
	int err = 0;

	// 1my_debug("%s.%d: %s IOCTL request\n", fimx6d.name, dev_num, __func__);

	switch (ioctl_num) {
		case FLEXCAN_IOCTL_READ:
			put_user(read_settings, (int *) ioctl_param);
			break;
		case FLEXCAN_IOCTL_WRITE:
			get_user(read_settings, (int *) ioctl_param);

			settings_debug = read_settings & SET_DRIVER_DEBUG_MASK;
			settings_mode = read_settings & SET_CAN_MODE_MASK;
			settings_bitrate = read_settings & SET_CAN_BITRATE_MASK;

			if(settings_debug)	{
				if(settings_debug & SET_DRIVER_DEBUG_ON)	{
					// 1my_debug = printk;
				}
				else if(settings_debug & SET_DRIVER_DEBUG_OFF)	{
					// 1my_debug = pp;
				}
			}
			if(settings_bitrate || settings_mode)	{
				switch (settings_bitrate) 	{
					case SET_CAN_BITRATE_1000:
						fimx6d.f_dev[dev_num].reg_ctrl_bittiming = FLEXCAN_BTRT_1000;
						break;
					case SET_CAN_BITRATE_500:
						fimx6d.f_dev[dev_num].reg_ctrl_bittiming = FLEXCAN_BTRT_500;
						break;
					case SET_CAN_BITRATE_250:
						fimx6d.f_dev[dev_num].reg_ctrl_bittiming = FLEXCAN_BTRT_250;
						break;
					case SET_CAN_BITRATE_125:
						fimx6d.f_dev[dev_num].reg_ctrl_bittiming = FLEXCAN_BTRT_125;
						break;
					case SET_CAN_BITRATE_100:
						fimx6d.f_dev[dev_num].reg_ctrl_bittiming = FLEXCAN_BTRT_100;
						break;
				}
				if (settings_mode & SET_CAN_MODE_LOOPBACK)	{
					fimx6d.f_dev[dev_num].reg_ctrl_bittiming |= FLEXCAN_CTRL_LPB;
				}
				if (settings_mode & SET_CAN_MODE_LISTENONLY)	{
					fimx6d.f_dev[dev_num].reg_ctrl_bittiming |= FLEXCAN_CTRL_LOM;
				}
				if (settings_mode & SET_CAN_MODE_3_SAMPLES)	{
					fimx6d.f_dev[dev_num].reg_ctrl_bittiming |= FLEXCAN_CTRL_SMP;
				}
				flexcan_chip_stop(dev_num);
				err = flexcan_chip_start(dev_num);
				if(err)	{
					// 1my_debug("%s.%d: %s error when try to start the module\n", fimx6d.name, dev_num, __func__);
				}
			}
			// тут вызвать функцию которая задаст новые настройки для конкретного can интерфейса
			// возможно нужно самому написать эту функцию, переписав flexcan_set_bittiming() или не нужно, еще не проверял.
			break;
		case FLEXCAN_IOCTL_WR_RD:
			// copy_from_user(&cf, (int *) ioctl_param, sizeof(struct can_frame));	
			// // 1my_debug("%s.%d: %s copy from user: DLC 0x%02x, ID %#08x, DATA = 0x%02x%02x%02x%02x 0x%02x%02x%02x%02x\n", 
			// 			fimx6d.name, dev_num, __func__, cf.can_dlc, cf.can_id, 
			// 			cf.data[0], cf.data[1], cf.data[2], cf.data[3], cf.data[4], cf.data[5], cf.data[6], cf.data[7]);
			// flexcan_transmit_frame(&cf, &dev_num);
			return -EINVAL;					// Пока не придумаю зачем надо будет не поддерживаемая инструкция
			break;
		default:
			return -EINVAL;
			break;
	}
	return 0;

/*
	unsigned int buf_ret = 0;
	buf_ret = flexcan_send_buf_is_empty(dev_num);
	if(buf_ret)	{
		flexcan_start_transmit(dev_num);
	}

*/

/*
	int i = 0, err = 0;
	char* temp;
	char charget;
	int* usr_send_ptr = 0;
	static int usr_buf_size = DEFAULT_USR_BUF_SIZE;
	static int drv_buf_size = CIRCLE_BUF_CAP;

	// printk("ioctl. file->f_owner->uid = %d, file->f_owner->euid = %d\n", (*file).f_owner.uid, (*file).f_owner.euid);
	// printk("ioctl. minor = %d, major = %d\n", iminor(file->f_path.dentry->d_inode), imajor(file->f_path.dentry->d_inode));

	switch (ioctl_num) {
		case FLEX_IOC_CMD_STOP:
			printk("%s: %s Ioctl=FLEX_IOC_CMD_STOP, NumOfReadFrames = %d\n", DRV_NAME, __func__, NumOfReadFrames);	
			break;

		case FLEX_IOC_SET_USR_BUF_SIZE:				// Записать в драйвер размер буфера пользователя для передачи данных
			usr_buf_size = (int)ioctl_param;
			flexcan_gpio_switch(fimx6d.f_dev[0].pdata, 0);
			flexcan_gpio_switch(fimx6d.f_dev[1].pdata, 0);
			break;

		case FLEX_IOC_SET_DRV_BUF_SIZE:				// Записать в драйвер размер циклического буфера для хранения CAN сообщений
			// drv_buf_size = (int)ioctl_param;
			// Пустая функция т.к. пока нет возможности задать размер циклического буфера после компиляции драйвера
			break;

		case FLEX_IOC_RD_USR_BUF_SIZE:				// Прочитать установленный в драйвере размер буфера пользователя
			put_user(usr_buf_size, (int *)ioctl_param);
			break;

		case FLEX_IOC_RD_DRV_BUF_SIZE:				// Прочитать установленный в драйвере размер циклического буфера драйвера
			put_user(drv_buf_size, (int *)ioctl_param);
			break;

		case FLEX_IOC_RD_DRV_BUF_CAP:				// Прочитать кол-во данных в буфере драйвера
			// put_user(flexcan_frame_buf_get_data_size(), (int *)ioctl_param);
			break;

		case FLEX_IOC_SEND_CMD_CHRP:
			flexcan_gpio_switch(fimx6d.f_dev[0].pdata, 1);
			flexcan_gpio_switch(fimx6d.f_dev[1].pdata, 1);
			// Пока пустая функция
			break;

		case FLEX_IOC_RD_STAT_CHRP:
			flexcan_gpio_switch(fimx6d.f_dev[0].pdata, 0);
			flexcan_gpio_switch(fimx6d.f_dev[1].pdata, 0);
			// i = flexcan_file_read(file, (char *)ioctl_param, 99, 0);
			// put_user('\0', (char *)ioctl_param + i);
			break;

		// case IOCTL_GET_NTH_BYTE:
		// 	return Message[ioctl_param];
		// 	break;

		case FLEX_IOC_RD_BUF_INTP:					// Прочитать содержимое циклического буфера и передать по указателю
			err = !access_ok(VERIFY_WRITE, (void *)ioctl_param, sizeof(int));
			if (err) {
				return -EFAULT;
			}
			else 	{
				i = flexcan_buffer_read(file, (int *)ioctl_param, usr_buf_size, 0);
				__put_user('\0', (int *)ioctl_param + i);
			}
			break;
	}
	return 0;
*/
}

static int flexcan_char_release(struct inode *i, struct file *file)
{
	u8 dev_num = iminor(i);
	fimx6d.f_dev[dev_num].is_open = 0;
	// 1my_debug("%s.%d: %s \n", fimx6d.name, dev_num, __func__);
	module_put(THIS_MODULE);
    return 0;
}

static int flexcan_char_open(struct inode *i, struct file *file)
{
	u8 dev_num = iminor(i);
	if(fimx6d.f_dev[dev_num].is_open)	{
		return -EBUSY;
		// 1my_debug("%s.%d: %s file was opened early\n", fimx6d.name, dev_num, __func__);
	}
	// 1my_debug("%s.%d: %s file open success\n", fimx6d.name, dev_num, __func__);

	fimx6d.f_dev[dev_num].is_open = 1;
	try_module_get(THIS_MODULE);
	return 0;
}

static struct file_operations flexcan_fops =
{
    .owner = 	THIS_MODULE,
    .open = 	flexcan_char_open,
    .release = 	flexcan_char_release,
    .read = 	flexcan_char_read,
    .write = 	flexcan_char_write,
	#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))
    	.ioctl = flexcan_char_ioctl,
	#else
    	.unlocked_ioctl = flexcan_char_ioctl,
	#endif
};
// ######################################################################################################################
// ######################################################################################################################

static inline void flexcan_f_dev_init(struct flexcan_chardev *f_dev)
{
	f_dev->stats.rx_frames = 0;
	f_dev->stats.tx_frames = 0;
	f_dev->stats.rx_bytes = 0;
	f_dev->stats.tx_bytes = 0;	

	f_dev->stats.int_wak = 0;
	f_dev->stats.int_state = 0;
	f_dev->stats.int_rx_frame = 0;
	f_dev->stats.int_tx_frame = 0;
	f_dev->stats.int_num = 0;

	f_dev->stats.err_tx = 0;
	f_dev->stats.err_rx = 0;
	f_dev->stats.err_over = 0;
	f_dev->stats.err_warn = 0;
	f_dev->stats.err_frame = 0;	
	f_dev->stats.err_drop = 0;
	f_dev->stats.err_length = 0;
	f_dev->stats.err_fifo = 0;

	f_dev->stats.reg_esr = 0;	
	f_dev->stats.reg_mcr = 0;	
	f_dev->stats.reg_ctrl = 0;

	f_dev->buf_recv_size = BUF_RECV_CAP;
	f_dev->frame_buf_recv.n_first = 0;
	f_dev->frame_buf_recv.n_last = 0;

	f_dev->buf_send_size = BUF_SEND_CAP;
	f_dev->frame_buf_send.n_first = 0;
	f_dev->frame_buf_send.n_last = 0;

	f_dev->reg_ctrl_bittiming = FLEXCAN_BTRT_DFLT;
}

// ######################################################################################################################
// ##############################	Чисто временная мера, потом вернуть код на место!!!	  ###############################
// ######################################################################################################################
// #include "code_proc.c"
#define NAME_DIR 		"flexcan"
#define LEN_MSG 3072


static char *get_rw_buf(const u8 dev_num, int *length) 
{
	struct flexcan_stats *stats = &fimx6d.f_dev[dev_num].stats;
	static char buf_msg[LEN_MSG + 1];
	char *buf_ptr = buf_msg;
	int buf_length = 0;
	const int max_digits = 10;

	memset(buf_msg, 0, sizeof(buf_msg));

	char new_line[] = "\n";
	char info_head_rxtx[] = "Data communication statistic:\n";
	memcpy(buf_ptr, &info_head_rxtx, sizeof(info_head_rxtx));
	buf_ptr += sizeof(info_head_rxtx);
	buf_length += sizeof(info_head_rxtx);
	char info_rx_frames[] = "\tReceived frames    = ";		// количество принятых фреймов
	memcpy(buf_ptr, &info_rx_frames, sizeof(info_rx_frames));
	buf_ptr += sizeof(info_rx_frames);
	buf_length += sizeof(info_rx_frames);
	sprintf(buf_ptr, "%d", (int) stats->rx_frames);
	buf_ptr += max_digits;	
	buf_length += max_digits;
	memcpy(buf_ptr, &new_line, sizeof(new_line));
	buf_ptr += sizeof(new_line);
	buf_length += sizeof(new_line);
	char info_tx_frames[] = "\tTransmited frames  = ";		// количество отправленных фреймов
	memcpy(buf_ptr, &info_tx_frames, sizeof(info_tx_frames));
	buf_ptr += sizeof(info_tx_frames);
	buf_length += sizeof(info_tx_frames);
	sprintf(buf_ptr, "%d", (int) stats->tx_frames);
	buf_ptr += max_digits;	
	buf_length += max_digits;	
	memcpy(buf_ptr, &new_line, sizeof(new_line));
	buf_ptr += sizeof(new_line);
	buf_length += sizeof(new_line);
	char info_rx_bytes[] = "\tReceived bytes   = ";			// количество принятых байт данных
	memcpy(buf_ptr, &info_rx_bytes, sizeof(info_rx_bytes));
	buf_ptr += sizeof(info_rx_bytes);
	buf_length += sizeof(info_rx_bytes);
	sprintf(buf_ptr, "%d", (int) stats->rx_bytes);
	buf_ptr += max_digits;
	buf_length += max_digits;		
	memcpy(buf_ptr, &new_line, sizeof(new_line));
	buf_ptr += sizeof(new_line);
	buf_length += sizeof(new_line);
	char info_tx_bytes[] = "\tTransmited bytes = ";		// количество отправленных байт данных
	memcpy(buf_ptr, &info_tx_bytes, sizeof(info_tx_bytes));
	buf_ptr += sizeof(info_tx_bytes);
	buf_length += sizeof(info_tx_bytes);
	sprintf(buf_ptr, "%d", stats->tx_bytes);
	buf_ptr += max_digits;
	buf_length += max_digits;	
	memcpy(buf_ptr, &new_line, sizeof(new_line));
	buf_ptr += sizeof(new_line);
	buf_length += sizeof(new_line);

	memcpy(buf_ptr, &new_line, sizeof(new_line));
	buf_ptr += sizeof(new_line);
	buf_length += sizeof(new_line);
	char info_head_int[] = "Interrupts statistic:\n";
	memcpy(buf_ptr, &info_head_int, sizeof(info_head_int));
	buf_ptr += sizeof(info_head_int);
	buf_length += sizeof(info_head_int);	
	char info_int_wak[] = "\tWakeUp interrupts         = ";				// количество прерываний пробуждения
	memcpy(buf_ptr, &info_int_wak, sizeof(info_int_wak));
	buf_ptr += sizeof(info_int_wak);
	buf_length += sizeof(info_int_wak);
	sprintf(buf_ptr, "%d", stats->int_wak);
	buf_ptr += max_digits;	
	buf_length += max_digits;	
	memcpy(buf_ptr, &new_line, sizeof(new_line));
	buf_ptr += sizeof(new_line);
	buf_length += sizeof(new_line);
	char info_int_state[] = "\tState change interrupts   = ";		// количество прерываний проверки состояния
	memcpy(buf_ptr, &info_int_state, sizeof(info_int_state));
	buf_ptr += sizeof(info_int_state);
	buf_length += sizeof(info_int_state);
	sprintf(buf_ptr, "%d", stats->int_state);
	buf_ptr += max_digits;	
	buf_length += max_digits;	
	memcpy(buf_ptr, &new_line, sizeof(new_line));
	buf_ptr += sizeof(new_line);
	buf_length += sizeof(new_line);
	char info_int_rx_frame[] = "\tRead frame interrupts     = ";		// количество прерываний чтения фреймов
	memcpy(buf_ptr, &info_int_rx_frame, sizeof(info_int_rx_frame));
	buf_ptr += sizeof(info_int_rx_frame);
	buf_length += sizeof(info_int_rx_frame);
	sprintf(buf_ptr, "%d", stats->int_rx_frame);
	buf_ptr += max_digits;	
	buf_length += max_digits;	
	memcpy(buf_ptr, &new_line, sizeof(new_line));
	buf_ptr += sizeof(new_line);
	buf_length += sizeof(new_line);
	char info_int_tx_frame[] = "\tTransmit frame interrupts = ";	// количество прерываний отправки фреймов
	memcpy(buf_ptr, &info_int_tx_frame, sizeof(info_int_tx_frame));
	buf_ptr += sizeof(info_int_tx_frame);
	buf_length += sizeof(info_int_tx_frame);
	sprintf(buf_ptr, "%d", stats->int_tx_frame);
	buf_ptr += max_digits;	
	buf_length += max_digits;		
	memcpy(buf_ptr, &new_line, sizeof(new_line));
	buf_ptr += sizeof(new_line);
	buf_length += sizeof(new_line);
	char info_int_num[] = "\tTotal interrupts count    = ";					// количество прерываний
	memcpy(buf_ptr, &info_int_num, sizeof(info_int_num));
	buf_ptr += sizeof(info_int_num);
	buf_length += sizeof(info_int_num);
	sprintf(buf_ptr, "%d", stats->int_num);
	buf_ptr += max_digits;	
	buf_length += max_digits;	
	memcpy(buf_ptr, &new_line, sizeof(new_line));
	buf_ptr += sizeof(new_line);
	buf_length += sizeof(new_line);

	memcpy(buf_ptr, &new_line, sizeof(new_line));
	buf_ptr += sizeof(new_line);
	buf_length += sizeof(new_line);
	char info_head_err[] = "Errors statistic:\n";
	memcpy(buf_ptr, &info_head_err, sizeof(info_head_err));
	buf_ptr += sizeof(info_head_err);
	buf_length += sizeof(info_head_err);
	char info_err_tx[] = "\tTransmit errors = ";			// количество ошибок отправки фреймов
	memcpy(buf_ptr, &info_err_tx, sizeof(info_err_tx));
	buf_ptr += sizeof(info_err_tx);
	buf_length += sizeof(info_err_tx);
	sprintf(buf_ptr, "%d", stats->err_tx);
	buf_ptr += max_digits;	
	buf_length += max_digits;
	memcpy(buf_ptr, &new_line, sizeof(new_line));
	buf_ptr += sizeof(new_line);
	buf_length += sizeof(new_line);
	char info_err_rx[] = "\tReceive errors  = ";			// количество ошибок принятия фреймов
	memcpy(buf_ptr, &info_err_rx, sizeof(info_err_rx));
	buf_ptr += sizeof(info_err_rx);
	buf_length += sizeof(info_err_rx);
	sprintf(buf_ptr, "%d", stats->err_rx);
	buf_ptr += max_digits;	
	buf_length += max_digits;
	memcpy(buf_ptr, &new_line, sizeof(new_line));
	buf_ptr += sizeof(new_line);
	buf_length += sizeof(new_line);
	char info_err_over[] = "\tOverflow errors = ";		// количество переполнений буффера
	memcpy(buf_ptr, &info_err_over, sizeof(info_err_over));
	buf_ptr += sizeof(info_err_over);
	buf_length += sizeof(info_err_over);
	sprintf(buf_ptr, "%d", stats->err_over);
	buf_ptr += max_digits;	
	buf_length += max_digits;
	memcpy(buf_ptr, &new_line, sizeof(new_line));
	buf_ptr += sizeof(new_line);
	buf_length += sizeof(new_line);
	char info_err_warn[] = "\tWarning errors  = ";			// количество заполнений буффера
	memcpy(buf_ptr, &info_err_warn, sizeof(info_err_warn));
	buf_ptr += sizeof(info_err_warn);
	buf_length += sizeof(info_err_warn);
	sprintf(buf_ptr, "%d", stats->err_warn);
	buf_ptr += max_digits;	
	buf_length += max_digits;
	memcpy(buf_ptr, &new_line, sizeof(new_line));
	buf_ptr += sizeof(new_line);
	buf_length += sizeof(new_line);
	char info_err_frame[] = "\tFrame errors    = ";			// количество ошибочных фреймов
	memcpy(buf_ptr, &info_err_frame, sizeof(info_err_frame));
	buf_ptr += sizeof(info_err_frame);
	buf_length += sizeof(info_err_frame);
	sprintf(buf_ptr, "%d", stats->err_frame);
	buf_ptr += max_digits;	
	buf_length += max_digits;
	memcpy(buf_ptr, &new_line, sizeof(new_line));
	buf_ptr += sizeof(new_line);
	buf_length += sizeof(new_line);
	char info_err_drop[] = "\tDroped errors   = ";			// количество потерянных при чтении фреймов
	memcpy(buf_ptr, &info_err_drop, sizeof(info_err_drop));
	buf_ptr += sizeof(info_err_drop);
	buf_length += sizeof(info_err_drop);
	sprintf(buf_ptr, "%d", stats->err_drop);
	buf_ptr += max_digits;	
	buf_length += max_digits;
	memcpy(buf_ptr, &new_line, sizeof(new_line));
	buf_ptr += sizeof(new_line);
	buf_length += sizeof(new_line);
	char info_err_length[] = "\tLength errors   = ";		// количество ошибок длины фреймов
	memcpy(buf_ptr, &info_err_length, sizeof(info_err_length));
	buf_ptr += sizeof(info_err_length);
	buf_length += sizeof(info_err_length);
	sprintf(buf_ptr, "%d", stats->err_length);
	buf_ptr += max_digits;	
	buf_length += max_digits;
	memcpy(buf_ptr, &new_line, sizeof(new_line));
	buf_ptr += sizeof(new_line);
	buf_length += sizeof(new_line);
	char info_err_fifo[] = "\tFIFO errors     = ";			// количество ошибок fifo буффера
	memcpy(buf_ptr, &info_err_fifo, sizeof(info_err_fifo));
	buf_ptr += sizeof(info_err_fifo);
	buf_length += sizeof(info_err_fifo);
	sprintf(buf_ptr, "%d", stats->err_fifo);
	buf_ptr += max_digits;	
	buf_length += max_digits;
	memcpy(buf_ptr, &new_line, sizeof(new_line));
	buf_ptr += sizeof(new_line);
	buf_length += sizeof(new_line);

	memcpy(buf_ptr, &new_line, sizeof(new_line));
	buf_ptr += sizeof(new_line);
	buf_length += sizeof(new_line);
	char info_head_reg[] = "Registers state:\n";
	memcpy(buf_ptr, &info_head_reg, sizeof(info_head_reg));
	buf_ptr += sizeof(info_head_reg);
	buf_length += sizeof(info_head_reg);
	char info_reg_esr[] = "\tESR register  = ";			// состояние регистра esr
	memcpy(buf_ptr, &info_reg_esr, sizeof(info_reg_esr));
	buf_ptr += sizeof(info_reg_esr);
	buf_length += sizeof(info_reg_esr);
	sprintf(buf_ptr, "0x%08x", stats->reg_esr);	
	buf_ptr += max_digits;	
	buf_length += max_digits;
	memcpy(buf_ptr, &new_line, sizeof(new_line));
	buf_ptr += sizeof(new_line);
	buf_length += sizeof(new_line);
	char info_reg_mcr[] = "\tMCR register  = ";			// состояние регистра mcr
	memcpy(buf_ptr, &info_reg_mcr, sizeof(info_reg_mcr));
	buf_ptr += sizeof(info_reg_mcr);
	buf_length += sizeof(info_reg_mcr);
	sprintf(buf_ptr, "0x%08x", stats->reg_mcr);	
	buf_ptr += max_digits;	
	buf_length += max_digits;
	memcpy(buf_ptr, &new_line, sizeof(new_line));
	buf_ptr += sizeof(new_line);
	buf_length += sizeof(new_line);
	char info_reg_ctrl[] = "\tCTRL register = ";			// состояние регистра ctrl
	memcpy(buf_ptr, &info_reg_ctrl, sizeof(info_reg_ctrl));
	buf_ptr += sizeof(info_reg_ctrl);
	buf_length += sizeof(info_reg_ctrl);
	sprintf(buf_ptr, "0x%08x", stats->reg_ctrl);
	buf_ptr += max_digits;	
	buf_length += max_digits;
	memcpy(buf_ptr, &new_line, sizeof(new_line));
	buf_ptr += sizeof(new_line);
	buf_length += sizeof(new_line);

	memcpy(buf_ptr, &new_line, sizeof(new_line));
	buf_ptr += sizeof(new_line);
	buf_length += sizeof(new_line);
	char info_head_freq[] = "Module frequency:\n";
	memcpy(buf_ptr, &info_head_freq, sizeof(info_head_freq));
	buf_ptr += sizeof(info_head_freq);
	buf_length += sizeof(info_head_freq);
	char info_freq[] = "\tFreq = ";						// частота модуля
	memcpy(buf_ptr, &info_freq, sizeof(info_freq));
	buf_ptr += sizeof(info_freq);
	buf_length += sizeof(info_freq);
	sprintf(buf_ptr, "%d", stats->freq);
	buf_ptr += max_digits;	
	buf_length += max_digits;
	memcpy(buf_ptr, &new_line, sizeof(new_line));
	buf_ptr += sizeof(new_line);
	buf_length += sizeof(new_line);

	*length = buf_length;

	return buf_msg;
}

// чтение из /proc/*my_dir*/*my_node* :
static ssize_t flexcan_proc_read(struct file *file, char *buf, size_t count, loff_t *ppos) 
{
	struct inode *inode = file->f_path.dentry->d_inode;
	u8 dev_num = iminor(inode);
	int buf_length = 0;
	char *buf_msg = get_rw_buf(dev_num, &buf_length);
	int res;
	// 1my_debug("%s.%d: %s count = %d, buf_length = %d, *ppos = %d\n", fimx6d.name, dev_num, __func__, count, buf_length, (int) *ppos);

	if(*ppos >= buf_length) 	{
		// 1my_debug("%s.%d: %s End of File\n", fimx6d.name, dev_num, __func__);
		return 0;
	}

	res = copy_to_user((void*)buf, buf_msg + *ppos, buf_length);
	*ppos += buf_length;
	// // res = copy_to_user((void*)buf, buf_msg + *ppos, count);
	// // res = copy_to_user((void*)buf, buf_msg + *ppos, 3072);
	// *ppos += strlen(buf_msg);
	// buf_msg += strlen(buf_msg);
	// buf_length -= strlen(buf_msg);		
	// // 1my_debug("%s.%d: %s return %d bytes\n", fimx6d.name, dev_num, __func__, count);

	// return count;
	return buf_length;
}

static const struct file_operations flexcan_proc_fops = {
   .owner = THIS_MODULE,
   .read  = flexcan_proc_read,
};

static int flexcan_proc_dir_init(void) 
{
	int ret;

	fimx6d.dev_proc_dir = create_proc_entry(NAME_DIR, S_IFDIR | S_IRWXUGO, NULL);
	if(NULL == fimx6d.dev_proc_dir) {
		ret = -ENOENT;
		// 1my_debug("%s: %s can't create directory /proc/%s\n", fimx6d.name, __func__, NAME_DIR);
		goto err_dir;
	}

	fimx6d.dev_proc_dir->uid = 0;
	fimx6d.dev_proc_dir->gid = 0;

	// 1my_debug("%s: %s /proc/%s installed\n", fimx6d.name, __func__, NAME_DIR);

	return 0;

 err_dir:
	return ret;
}

static int flexcan_proc_file_init(u8 dev_num) 
{
	int ret;

	fimx6d.f_dev[dev_num].dev_proc_file = create_proc_entry(fimx6d.f_dev[dev_num].name, S_IFREG | S_IRUGO, fimx6d.dev_proc_dir);
	if(NULL == fimx6d.f_dev[dev_num].dev_proc_file) {
		ret = -ENOENT;
		// 1my_debug("%s.%d: %s can't create node /proc/%s/%s\n", fimx6d.name, dev_num, __func__, NAME_DIR, fimx6d.f_dev[dev_num].name);
		goto err_file;
	}

	fimx6d.f_dev[dev_num].dev_proc_file->uid = 0;
	fimx6d.f_dev[dev_num].dev_proc_file->gid = 0;
	fimx6d.f_dev[dev_num].dev_proc_file->proc_fops = &flexcan_proc_fops;

	// 1my_debug("%s.%d: %s /proc/%s/%s installed\n", fimx6d.name, dev_num, __func__, NAME_DIR, fimx6d.f_dev[dev_num].name);

	return 0;

 err_file:
	return ret;
}

static void flexcan_proc_file_exit(u8 dev_num) 
{
	remove_proc_entry(fimx6d.f_dev[dev_num].name, fimx6d.dev_proc_dir);
	// 1my_debug("%s.%d: %s /proc/%s/%s removed\n", fimx6d.name, dev_num, __func__, NAME_DIR, fimx6d.f_dev[dev_num].name);
}

static void flexcan_proc_dir_exit(void) 
{
	remove_proc_entry(NAME_DIR, NULL);
	// 1my_debug("%s: %s /proc/%s removed\n", fimx6d.name, __func__, NAME_DIR);
}

// ######################################################################################################################
// ######################################################################################################################


static void flexcan_transceiver_switch(const struct flexcan_platform_data *pdata, int on)
{
	if (pdata && pdata->transceiver_switch)	{
		pdata->transceiver_switch(on);
	}
}

static void flexcan_gpio_switch(const struct flexcan_platform_data *pdata, int on)
{
	if (pdata && pdata->gpio_switch)	{
		pdata->gpio_switch(on);
	}
}

static inline int flexcan_has_and_handle_berr(const struct flexcan_stats *stats, u32 reg_esr)
{
	return (stats->ctrlmode & CAN_CTRLMODE_BERR_REPORTING) && (reg_esr & FLEXCAN_ESR_ERR_BUS);
}

static inline void flexcan_chip_enable(void __iomem *base)
{
	struct flexcan_regs __iomem *regs = base;
	u32 reg;

	reg = readl(&regs->mcr);
	reg &= ~FLEXCAN_MCR_MDIS;
	writel(reg, &regs->mcr);

	udelay(10);
}

static inline void flexcan_chip_disable(void __iomem *base)
{
	struct flexcan_regs __iomem *regs = base;
	u32 reg;

	reg = readl(&regs->mcr);
	reg |= FLEXCAN_MCR_MDIS;
	writel(reg, &regs->mcr);
}

static int flexcan_get_berr_counter(u8 dev_num, struct can_berr_counter *bec)
{
	struct flexcan_regs __iomem *regs = fimx6d.f_dev[dev_num].f_base;
	u32 reg = readl(&regs->ecr);

	bec->txerr = (reg >> 0) & 0xff;
	bec->rxerr = (reg >> 8) & 0xff;

	return 0;
}

static unsigned int flexcan_start_transmit(u8 dev_num)
{
	struct can_frame cf;
	struct flexcan_regs __iomem *regs = fimx6d.f_dev[dev_num].f_base;
	struct flexcan_stats *stats = &fimx6d.f_dev[dev_num].stats;
	u32 can_id, ctrl;
	unsigned int buf_ret = 0;

	buf_ret = flexcan_send_buf_is_empty(dev_num);
	if(buf_ret)	{
		return 0;
	}

	/* На будущее проверка статуса буфера в данный момент */
	// u8 mb_code = ((readl(&regs->cantxfg[FLEXCAN_TX_BUF_ID].can_ctrl) >> 24) & 0x0F);

	buf_ret = flexcan_send_buf_pop(dev_num, &cf);
	if(buf_ret)	{
		return 0;
	}

	// 1my_debug("%s.%d: %s have to send: %#08x %#02x [0x%02x%02x%02x%02x,0x%02x%02x%02x%02x]\n", fimx6d.name, dev_num, __func__, cf.can_id, cf.can_dlc, cf.data[0], cf.data[1], cf.data[2], cf.data[3], cf.data[4], cf.data[5], cf.data[6], cf.data[7]);


	ctrl = FLEXCAN_MB_CNT_CODE(0xc) | (cf.can_dlc << 16);

	if (cf.can_id & CAN_EFF_FLAG) {
		can_id = cf.can_id & CAN_EFF_MASK;
		ctrl |= FLEXCAN_MB_CNT_IDE | FLEXCAN_MB_CNT_SRR;
	} 
	else {
		can_id = (cf.can_id & CAN_SFF_MASK) << 18;
	}

	if (cf.can_id & CAN_RTR_FLAG)	{
		ctrl |= FLEXCAN_MB_CNT_RTR;
	}

	if (cf.can_dlc > 0) {
		u32 data = be32_to_cpup((__be32 *)&cf.data[0]);
		writel(data, &regs->cantxfg[FLEXCAN_TX_BUF_ID].data[0]);
	}	
	if (cf.can_dlc > 3) {
		u32 data = be32_to_cpup((__be32 *)&cf.data[4]);
		writel(data, &regs->cantxfg[FLEXCAN_TX_BUF_ID].data[1]);
	}

	writel(can_id, &regs->cantxfg[FLEXCAN_TX_BUF_ID].can_id);
	writel(ctrl, &regs->cantxfg[FLEXCAN_TX_BUF_ID].can_ctrl);

	/* tx_packets is incremented in flexcan_irq */
	stats->tx_bytes += cf.can_dlc;

	return 0;
}

static void do_bus_err(struct can_frame *cf, u32 reg_esr, u8 dev_num)
{
	struct flexcan_stats *stats = &fimx6d.f_dev[dev_num].stats;
	int rx_errors = 0, tx_errors = 0;

	cf->can_id |= CAN_ERR_PROT | CAN_ERR_BUSERROR;

	if (reg_esr & FLEXCAN_ESR_BIT1_ERR) {
		// 1my_debug("%s.%d: %s BIT1_ERR irq\n", fimx6d.name, dev_num, __func__);
		cf->data[2] |= CAN_ERR_PROT_BIT1;
		tx_errors = 1;
	}
	if (reg_esr & FLEXCAN_ESR_BIT0_ERR) {
		// 1my_debug("%s.%d: %s BIT0_ERR irq\n", fimx6d.name, dev_num, __func__);
		cf->data[2] |= CAN_ERR_PROT_BIT0;
		tx_errors = 1;
	}
	if (reg_esr & FLEXCAN_ESR_ACK_ERR) {
		// 1my_debug("%s.%d: %s ACK_ERR irq\n", fimx6d.name, dev_num, __func__);
		cf->can_id |= CAN_ERR_ACK;
		cf->data[3] |= CAN_ERR_PROT_LOC_ACK;
		tx_errors = 1;
	}
	if (reg_esr & FLEXCAN_ESR_CRC_ERR) {
		// 1my_debug("%s.%d: %s CRC_ERR irq\n", fimx6d.name, dev_num, __func__);
		cf->data[2] |= CAN_ERR_PROT_BIT;
		cf->data[3] |= CAN_ERR_PROT_LOC_CRC_SEQ;
		rx_errors = 1;
	}
	if (reg_esr & FLEXCAN_ESR_FRM_ERR) {
		// 1my_debug("%s.%d: %s FRM_ERR irq\n", fimx6d.name, dev_num, __func__);
		cf->data[2] |= CAN_ERR_PROT_FORM;
		rx_errors = 1;
	}
	if (reg_esr & FLEXCAN_ESR_STF_ERR) {
		// 1my_debug("%s.%d: %s STF_ERR irq\n", fimx6d.name, dev_num, __func__);
		cf->data[2] |= CAN_ERR_PROT_STUFF;
		rx_errors = 1;
	}

	stats->dev_stats.bus_error++;
	if (rx_errors)	{
		stats->err_rx++;
	}
	if (tx_errors)	{
		stats->err_tx++;
	}
}

static void do_state(struct can_frame *cf, u8 dev_num, enum can_state new_state)
{
	struct flexcan_stats *stats = &fimx6d.f_dev[dev_num].stats;
	struct can_berr_counter bec;

	// 1my_debug("%s.%d: %s get berr counter\n", fimx6d.name, dev_num, __func__);
	flexcan_get_berr_counter(dev_num, &bec);

	switch (stats->state) {
	case CAN_STATE_ERROR_ACTIVE:
		/*
		 * from: ERROR_ACTIVE
		 * to  : ERROR_WARNING, ERROR_PASSIVE, BUS_OFF
		 * =>  : there was a warning int
		 */
		if (new_state >= CAN_STATE_ERROR_WARNING &&
		    new_state <= CAN_STATE_BUS_OFF) {
			// 1my_debug("%s.%d: %s Error Warning IRQ\n", fimx6d.name, dev_num, __func__);
			stats->dev_stats.error_warning++;

			cf->can_id |= CAN_ERR_CRTL;
			cf->data[1] = (bec.txerr > bec.rxerr) ?
				CAN_ERR_CRTL_TX_WARNING :
				CAN_ERR_CRTL_RX_WARNING;
		}
	case CAN_STATE_ERROR_WARNING:	/* fallthrough */
		/*
		 * from: ERROR_ACTIVE, ERROR_WARNING
		 * to  : ERROR_PASSIVE, BUS_OFF
		 * =>  : error passive int
		 */
		if (new_state >= CAN_STATE_ERROR_PASSIVE &&
		    new_state <= CAN_STATE_BUS_OFF) {
			// 1my_debug("%s.%d: %s Error Passive IRQ\n", fimx6d.name, dev_num, __func__);
			stats->dev_stats.error_passive++;

			cf->can_id |= CAN_ERR_CRTL;
			cf->data[1] = (bec.txerr > bec.rxerr) ?
				CAN_ERR_CRTL_TX_PASSIVE :
				CAN_ERR_CRTL_RX_PASSIVE;
		}
		break;
	case CAN_STATE_BUS_OFF:
		// 1my_debug("%s.%d: %s BUG! hardware recovered automatically from BUS_OFF\n", fimx6d.name, dev_num, __func__);
		break;
	default:
		break;
	}

	/* process state changes depending on the new state */
	switch (new_state) {
	case CAN_STATE_ERROR_ACTIVE:
		// 1my_debug("%s.%d: %s Error Active\n", fimx6d.name, dev_num, __func__);
		cf->can_id |= CAN_ERR_PROT;
		cf->data[2] = CAN_ERR_PROT_ACTIVE;
		break;
	case CAN_STATE_BUS_OFF:
		cf->can_id |= CAN_ERR_BUSOFF;
		// can_bus_off(dev);
		stats->dev_stats.bus_off++;
		break;
	default:
		break;
	}
}

static irqreturn_t flexcan_irq(int irq, void *dev_id)
{
	u8 dev_num = (irq - IRQ_BASE);

	struct flexcan_stats *stats = &fimx6d.f_dev[dev_num].stats;
	struct flexcan_regs __iomem *regs = fimx6d.f_dev[dev_num].f_base;

	struct flexcan_mb __iomem *mb = &regs->cantxfg[0];
	struct timeval time;
	u32 reg_iflag1, reg_esr;
	u32 i = 0;

	// flexcan_transceiver_switch(fimx6d.f_dev[dev_num].pdata, 1);

	reg_iflag1 = readl(&regs->iflag1);
	reg_esr = readl(&regs->esr);
	// // 1my_debug("%s.%d: %s interrupt request, reg_iflag1 = %#08x\n", fimx6d.name, dev_num, __func__, reg_iflag1);

	/* Считаем прерывания */
	stats->int_num++;

	if (reg_esr & FLEXCAN_ESR_WAK_INT) {
		if (fimx6d.version >= FLEXCAN_VER_10_0_12)	{
			mxc_iomux_set_gpr_register(13, 28, 1, 0);
		}
		writel(FLEXCAN_ESR_WAK_INT, &regs->esr);
	}

	if (reg_iflag1 & FLEXCAN_IFLAG_DEFAULT)	{
		if(reg_iflag1 & FLEXCAN_IFLAG_RX_FIFO_OVERFLOW)	{
			stats->err_over++;
			stats->err_drop++;
		}
		else if(reg_iflag1 & FLEXCAN_IFLAG_RX_FIFO_WARN)	{
			stats->err_warn++;
		}
			stats->int_rx_frame++;

		// // 1my_debug("%s.%d: read buffer contain data: CTRL %#08x, ID %#08x, DATA0 %#08x, DATA1 %#08x\n", 
		// 			fimx6d.name, dev_num, mb->can_ctrl, mb->can_id, mb->data[0], mb->data[1]);

		i = 0;
		while((i < 10) && (reg_iflag1 & FLEXCAN_IFLAG_RX_FIFO_AVAILABLE))	{	
			do_gettimeofday(&time);
			stats->rx_bytes += ((mb->can_ctrl >> 16) & 0xf);
			if(flexcan_recv_buf_push(dev_num, mb, &time))	{
				/* буффер заполнен */
				stats->err_drop++;
				stats->err_fifo++;
			}
			else 	{
				/* все хорошо */
				stats->rx_frames++;
			}

			if(reg_iflag1 & FLEXCAN_IFLAG_RX_FIFO_OVERFLOW)	{
				writel((FLEXCAN_IFLAG_DEFAULT), &regs->iflag1);
			}
			else if(reg_iflag1 & FLEXCAN_IFLAG_RX_FIFO_WARN)	{
				writel((FLEXCAN_IFLAG_RX_FIFO_WARN | FLEXCAN_IFLAG_RX_FIFO_AVAILABLE), &regs->iflag1);
			}
			else if(reg_iflag1 & FLEXCAN_IFLAG_RX_FIFO_AVAILABLE)	{
				writel(FLEXCAN_IFLAG_RX_FIFO_AVAILABLE, &regs->iflag1);
			}

			reg_iflag1 = readl(&regs->iflag1);
			readl(&regs->timer);
			i++;
		}
	}

	if ((reg_esr & FLEXCAN_ESR_ERR_STATE) || (flexcan_has_and_handle_berr(stats, reg_esr)))	{
		stats->int_state++;

		stats->reg_esr = reg_esr & FLEXCAN_ESR_ERR_BUS;
		writel(reg_esr, &regs->esr);
		writel(fimx6d.f_dev[dev_num].reg_ctrl_default, &regs->ctrl);			
	}

	if (reg_iflag1 & (1 << FLEXCAN_TX_BUF_ID)) {			/* transmission complete interrupt */
		/* tx_bytes is incremented in flexcan_start_xmit */
		stats->int_tx_frame++;
		stats->tx_frames++;
		writel((1 << FLEXCAN_TX_BUF_ID), &regs->iflag1);
		// // 1my_debug("%s.%d: transmit buffer was contain data: CTRL %#08x, ID %#08x, DATA0 %#08x, DATA1 %#08x\n", 
		// 			fimx6d.name, dev_num, mb->can_ctrl, mb->can_id, mb->data[0], mb->data[1]);
	}

	// flexcan_transceiver_switch(fimx6d.f_dev[dev_num].pdata, 0);

	return IRQ_HANDLED;
}




static void flexcan_set_bittiming(const u8 dev_num, const u32 reg_ctrl)
{
	struct flexcan_stats *stats = &fimx6d.f_dev[dev_num].stats;
	struct flexcan_regs __iomem *regs = fimx6d.f_dev[dev_num].f_base;
	u32 reg;

	reg = readl(&regs->ctrl);

	reg &= ~(FLEXCAN_BTRT_MASK | FLEXCAN_CTRL_LPB | FLEXCAN_CTRL_SMP | FLEXCAN_CTRL_LOM);
	reg |= reg_ctrl;

	// 1my_debug("%s.%d: %s writing ctrl=0x%08x\n", fimx6d.name, dev_num, __func__, reg);
	writel(reg, &regs->ctrl);
	stats->reg_mcr = readl(&regs->mcr);
	stats->reg_ctrl = readl(&regs->ctrl);

	/* print chip status */
	// 1my_debug("%s.%d: %s mcr=0x%08x ctrl=0x%08x\n", fimx6d.name, dev_num, __func__, readl(&regs->mcr), readl(&regs->ctrl));
}



static int flexcan_chip_start(const u8 dev_num)
{
	struct flexcan_stats *stats = &fimx6d.f_dev[dev_num].stats;
	struct flexcan_regs __iomem *regs = fimx6d.f_dev[dev_num].f_base;
	unsigned int i;
	int err;
	u32 reg_mcr, reg_ctrl;

	/* enable module */
	// 1my_debug("%s.%d: %s enable module\n", fimx6d.name, dev_num, __func__);
	flexcan_chip_enable(fimx6d.f_dev[dev_num].f_base);

	/* soft reset */
	// 1my_debug("%s.%d: %s soft reset\n", fimx6d.name, dev_num, __func__);
	writel(FLEXCAN_MCR_SOFTRST, &regs->mcr);
	udelay(10);

	reg_mcr = readl(&regs->mcr);
	if (reg_mcr & FLEXCAN_MCR_SOFTRST) {
		// 1my_debug("%s.%d: %s failed to softreset can module, mcr=0x%08x\n", fimx6d.name, dev_num, __func__, reg_mcr);
		err = -ENODEV;
		goto out;
	}

	// 1my_debug("%s.%d: %s set bittiming\n", fimx6d.name, dev_num, __func__);
	flexcan_set_bittiming(dev_num, fimx6d.f_dev[dev_num].reg_ctrl_bittiming);

	/* MCR
	 * enable freeze | enable fifo | halt now  | only supervisor access  | 
	 * enable warning int |  choose format C  | enable self wakeup
	 */
	reg_mcr = readl(&regs->mcr);
	reg_mcr |= FLEXCAN_MCR_FRZ | FLEXCAN_MCR_FEN | FLEXCAN_MCR_HALT | FLEXCAN_MCR_SUPV | 
		FLEXCAN_MCR_WRN_EN | FLEXCAN_MCR_IDAM_C | FLEXCAN_MCR_WAK_MSK | FLEXCAN_MCR_SLF_WAK;
	
	// 1my_debug("%s.%d: %s writing mcr=0x%08x\n", fimx6d.name, dev_num, __func__, reg_mcr);
	writel(reg_mcr, &regs->mcr);
	stats->reg_mcr = readl(&regs->mcr);
	/* CTRL
	 * disable timer sync feature | disable auto busoff recovery | transmit lowest buffer first
	 * enable tx and rx warning interrupt | enable bus off interrupt (== FLEXCAN_CTRL_ERR_STATE)
	 * _note_: we enable the "error interrupt" (FLEXCAN_CTRL_ERR_MSK), too. Otherwise we don't get any
	 * Otherwise we don't get any warning or bus passive interrupts.
	 */
	reg_ctrl = readl(&regs->ctrl);
	reg_ctrl &= ~FLEXCAN_CTRL_TSYN;
	// reg_ctrl |= /*FLEXCAN_CTRL_BOFF_REC | */ FLEXCAN_CTRL_LBUF |/*| FLEXCAN_CTRL_ERR_STATE | */FLEXCAN_CTRL_ERR_MSK ;
	reg_ctrl |= FLEXCAN_CTRL_BOFF_REC | FLEXCAN_CTRL_LBUF;

	/* save for later use */
	fimx6d.f_dev[dev_num].reg_ctrl_default = reg_ctrl;

	// 1my_debug("%s.%d: %s writing ctrl=0x%08x\n", fimx6d.name, dev_num, __func__, reg_ctrl);
	writel(reg_ctrl, &regs->ctrl);
	stats->reg_ctrl = readl(&regs->ctrl);

	for (i = 0; i < ARRAY_SIZE(regs->cantxfg); i++) {
		writel(0, &regs->cantxfg[i].can_ctrl);
		writel(0, &regs->cantxfg[i].can_id);
		writel(0, &regs->cantxfg[i].data[0]);
		writel(0, &regs->cantxfg[i].data[1]);
		/* put MB into rx queue */
		writel(FLEXCAN_MB_CNT_CODE(0x4), &regs->cantxfg[i].can_ctrl);
	}
	/* acceptance mask/acceptance code (accept everything) */
	writel(0x0, &regs->rxgmask);
	writel(0x0, &regs->rx14mask);
	writel(0x0, &regs->rx15mask);

	/* clear rx fifo global mask */
	if (fimx6d.version >= FLEXCAN_VER_10_0_12)	{
		writel(0x0, &regs->rxfgmask);
	}
	// flexcan_transceiver_switch(priv, 1);						//заменить priv  на pdata из структуры вызывающего устройства

	/* synchronize with the can bus */
	reg_mcr = readl(&regs->mcr);
	reg_mcr &= ~FLEXCAN_MCR_HALT;
	writel(reg_mcr, &regs->mcr);
	stats->reg_mcr = readl(&regs->mcr);

	stats->state = CAN_STATE_ERROR_ACTIVE;

	/* enable FIFO interrupts */
	writel(FLEXCAN_IFLAG_DEFAULT, &regs->imask1);

	/* print chip status */
	// 1my_debug("%s.%d: %s reading mcr=0x%08x ctrl=0x%08x\n", fimx6d.name, dev_num, __func__, readl(&regs->mcr), readl(&regs->ctrl));

	// // 1my_debug("%s.%d: %s registereng /proc file\n", fimx6d.name, dev_num, __func__);
	flexcan_proc_file_init(dev_num);

	return 0;

 out:
 	// 1my_debug("%s.%d: %s chip disable\n", fimx6d.name, dev_num, __func__);
	flexcan_chip_disable(fimx6d.f_dev[dev_num].f_base);
	return err;
}


static void flexcan_chip_stop(const u8 dev_num)
{
	struct flexcan_stats *stats = &fimx6d.f_dev[dev_num].stats;
	struct flexcan_regs __iomem *regs = fimx6d.f_dev[dev_num].f_base;
	u32 reg;

	// 1my_debug("%s.%d: %s was call with dev_num = %d\n", fimx6d.name, dev_num, __func__, dev_num);
	
	regs = fimx6d.f_dev[dev_num].f_base;
	// // 1my_debug("%s.%d: %s unregistereng /proc file\n", fimx6d.name, dev_num, __func__);
	flexcan_proc_file_exit(dev_num);

	/* Disable all interrupts */
	writel(0, &regs->imask1);

	/* Disable + halt module */
	reg = readl(&regs->mcr);
	reg |= FLEXCAN_MCR_MDIS | FLEXCAN_MCR_HALT;
	writel(reg, &regs->mcr);
	stats->reg_mcr = readl(&regs->mcr);

	// flexcan_transceiver_switch(priv, 0);						//заменить priv  на pdata из структуры вызывающего устройства
	stats->state = CAN_STATE_STOPPED;
}

static int flexcan_net_open(const u8 dev_num)
{
	unsigned int irq = fimx6d.f_dev[dev_num].irq_num;
	int err;

	// 1my_debug("%s.%d: %s clk enable\n", fimx6d.name, dev_num, __func__);
	clk_enable(fimx6d.f_dev[dev_num].clk);

	err = request_irq(irq, flexcan_irq, IRQF_SHARED, fimx6d.f_dev[dev_num].name, fimx6d.f_dev[dev_num].pdata);
	// err = request_irq(irq, flexcan_irq, IRQF_SHARED, fimx6d.f_dev[dev_num].name, &fimx6d.f_dev[dev_num].f_cdev);
	if (err)	{
		// 1my_debug("%s.%d: %s request irq %d failed with err = %d\n", fimx6d.name, dev_num, __func__, irq, err);
		goto out;
	}
	else 	{
		// 1my_debug("%s.%d: %s request irq %d success\n", fimx6d.name, dev_num, __func__, irq);
	}
	/* start chip and queuing */
	// 1my_debug("%s.%d: %s chip start\n", fimx6d.name, dev_num, __func__);
	err = flexcan_chip_start(dev_num);
	if (err)	{
		goto out;
	}

	return 0;

 out:
 	// 1my_debug("%s.%d: %s goto out. clk disable\n", fimx6d.name, dev_num, __func__);
	clk_disable(fimx6d.f_dev[dev_num].clk);

	return err;
}

static int flexcan_net_close(const u8 dev_num)
{
	unsigned int irq = fimx6d.f_dev[dev_num].irq_num;

	// 1my_debug("%s.%d: %s chip stop\n", fimx6d.name, dev_num, __func__);
	flexcan_chip_stop(dev_num);

	// 1my_debug("%s.%d: %s free irq\n", fimx6d.name, dev_num, __func__);
	// free_irq(irq, &fimx6d.f_dev[dev_num].f_cdev);
	free_irq(irq, fimx6d.f_dev[dev_num].pdata);

	// 1my_debug("%s.%d: %s clk disable\n", fimx6d.name, dev_num, __func__);
	clk_disable(fimx6d.f_dev[dev_num].clk);

	return 0;
}

static int flexcan_set_mode(const u8 dev_num, enum can_mode mode)
{
	int err;

	switch (mode) {
	case CAN_MODE_START:
		err = flexcan_chip_start(dev_num);
		if (err)	{
			return err;
		}
		// netif_wake_queue(dev);
		// и зачем мне эта функция теперь?
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static int __devinit register_flexcandev(const u8 dev_num)
{
	struct flexcan_regs __iomem *regs = fimx6d.f_dev[dev_num].f_base;
	u32 reg, err;

	clk_enable(fimx6d.f_dev[dev_num].clk);
	// 1my_debug("%s.%d: %s clk enable\n", fimx6d.name, dev_num, __func__);

	/* select "bus clock", chip must be disabled */
	flexcan_chip_disable(fimx6d.f_dev[dev_num].f_base);
	reg = readl(&regs->ctrl);
	reg |= FLEXCAN_CTRL_CLK_SRC;
	writel(reg, &regs->ctrl);

	flexcan_chip_enable(fimx6d.f_dev[dev_num].f_base);
	// 1my_debug("%s.%d: %s chip enable\n", fimx6d.name, dev_num, __func__);
	/* set freeze, halt and activate FIFO, restrict register access */
	reg = readl(&regs->mcr);
	reg |= FLEXCAN_MCR_FRZ | FLEXCAN_MCR_HALT | FLEXCAN_MCR_FEN | FLEXCAN_MCR_SUPV;
	writel(reg, &regs->mcr);

	/* Currently we only support newer versions of this core
	 * featuring a RX FIFO. Older cores found on some Coldfire
	 * derivates are not yet supported.
	 */
	reg = readl(&regs->mcr);
	if (!(reg & FLEXCAN_MCR_FEN)) {
		// 1my_debug("%s.%d: %s Could not enable RX FIFO, unsupported core\n", fimx6d.name, dev_num, __func__);
		err = -ENODEV;
		goto out;
	}

 out:
	/* disable core and turn off clocks */
 	flexcan_chip_disable(fimx6d.f_dev[dev_num].f_base);
	clk_disable(fimx6d.f_dev[dev_num].clk);

	return err;
}

static struct platform_device_id flexcan_devtype[] = {
	{
		.name = "imx25-flexcan",
		.driver_data = FLEXCAN_VER_3_0_0,
	}, {
		.name = "imx28-flexcan",
		.driver_data = FLEXCAN_VER_3_0_4,
	}, {
		.name = "imx35-flexcan",
		.driver_data = FLEXCAN_VER_3_0_0,
	}, {
		.name = "imx53-flexcan",
		.driver_data = FLEXCAN_VER_3_0_0,
	}, {
		.name = "imx6q-flexcan",
		.driver_data = FLEXCAN_VER_10_0_12,
	},
};

static int __devinit flexcan_probe(struct platform_device *pdev)
{
	struct resource *mem;
	struct clk *clk;
	void __iomem *base;
	resource_size_t mem_size;
	int err, irq, major;
	char name0[IFNAMSIZ] = "can0";
	char name1[IFNAMSIZ] = "can1";

	printk("%s driver ver %s by Strim-tech\n", DRV_NAME, DRV_VER);

	/* pdev->id 	- ID устройства (0 или 1)
	 * dev->irq 	- IRQ number
	 * dev->name 	- Имя устройства (can0 или can1)
	 * pdev->id_entry->name - Имя родителя (imx6d-flexcan)
	 */

	clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "no clock defined\n");	
		err = PTR_ERR(clk);
		goto failed_clock;
	}

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq = platform_get_irq(pdev, 0);
	if (!mem || irq <= 0) {
		err = -ENODEV;
		goto failed_get;
	}

	mem_size = resource_size(mem);
	if (!request_mem_region(mem->start, mem_size, pdev->name)) {
		err = -EBUSY;
		goto failed_get;
	}

	base = ioremap(mem->start, mem_size);
	if (!base) {
		err = -ENOMEM;
		goto failed_map;
	}

	flexcan_f_dev_init(&(fimx6d.f_dev[pdev->id]));
	fimx6d.f_dev[pdev->id].irq_num = irq;
	fimx6d.f_dev[pdev->id].stats.freq = clk_get_rate(clk);
	fimx6d.f_dev[pdev->id].stats.bittiming_const = &flexcan_bittiming_const;
	fimx6d.f_dev[pdev->id].do_set_mode = flexcan_set_mode;			// вернуть когда буду убирать netdevice
	fimx6d.f_dev[pdev->id].do_get_berr_counter = flexcan_get_berr_counter;
	fimx6d.f_dev[pdev->id].stats.ctrlmode_supported = CAN_CTRLMODE_LOOPBACK | CAN_CTRLMODE_LISTENONLY | CAN_CTRLMODE_3_SAMPLES | CAN_CTRLMODE_BERR_REPORTING;
	fimx6d.f_dev[pdev->id].f_base = base;
	fimx6d.f_dev[pdev->id].clk = clk;
	fimx6d.f_dev[pdev->id].pdata = pdev->dev.platform_data;
	fimx6d.version = pdev->id_entry->driver_data;
	memcpy(fimx6d.name, pdev->id_entry->name, sizeof(fimx6d.name));
	
    if(pdev->id == 0)	{
		memcpy(fimx6d.f_dev[pdev->id].name, name0, sizeof(fimx6d.f_dev[pdev->id].name));
    }
    else if(pdev->id == 1) 	{
    	memcpy(fimx6d.f_dev[pdev->id].name, name1, sizeof(fimx6d.f_dev[pdev->id].name));
    }
    // 1my_debug("%s.%d: %s device name = %s\n",fimx6d.name, pdev->id, __func__, fimx6d.f_dev[pdev->id].name);

	if(!fimx6d.is_init)	{
		err = alloc_chrdev_region(&fimx6d.f_devt, DEV_FIRST, DEV_COUNT, fimx6d.name);
		if (err < 0) {
			err = -1;
			// 1my_debug("%s.%d: %s alloc chrdev region failed\n",fimx6d.name, pdev->id, __func__);
			goto err_reg_chrdev;
		} 
		else 	{
			// 1my_debug("%s.%d: %s alloc chrdev region success, major = %d\n",fimx6d.name, pdev->id, __func__, MAJOR(fimx6d.f_devt));	
		}
	}
	major = MAJOR(fimx6d.f_devt);
	// 1my_debug("%s.%d: %s major = %d, pdev->id = %d\n",fimx6d.name, pdev->id, __func__, major, pdev->id);	

	fimx6d.f_dev[pdev->id].f_devt = MKDEV(major, pdev->id);  
			
	// 1my_debug("%s.%d: %s c_dev init\n",fimx6d.name, pdev->id, __func__);	
	cdev_init(&fimx6d.f_dev[pdev->id].f_cdev, &flexcan_fops);
	fimx6d.f_dev[pdev->id].f_cdev.owner = THIS_MODULE;
	fimx6d.f_dev[pdev->id].f_cdev.ops = &flexcan_fops;

	err = cdev_add(&fimx6d.f_dev[pdev->id].f_cdev, fimx6d.f_dev[pdev->id].f_devt, 1);
	if(err)	{
		err = -1;
		// 1my_debug("%s.%d: %s c_dev add error\n",fimx6d.name, pdev->id, __func__);
		goto err_cdev_add;
	}
	else 	{
		// 1my_debug("%s.%d: %s c_dev was added\n",fimx6d.name, pdev->id, __func__);
	}

	if(!fimx6d.is_init)	{
	    if ((fimx6d.f_class = class_create(THIS_MODULE, fimx6d.name)) == NULL)	{
			err = -1;
			// 1my_debug("%s.%d: %s class create error\n",fimx6d.name, pdev->id, __func__);
			goto err_class_create;
		}
		else 	{
			flexcan_proc_dir_init();
			// 1my_debug("%s.%d: %s class was create with name = %s\n",fimx6d.name, pdev->id, __func__, fimx6d.name);
		}
	}

	if (device_create(fimx6d.f_class, &pdev->dev, fimx6d.f_dev[pdev->id].f_devt, NULL, fimx6d.f_dev[pdev->id].name) == NULL)	{
		err = -1;
		// 1my_debug("%s.%d: %s device create error\n",fimx6d.name, pdev->id, __func__);
		goto err_device_create;
	}
	else 	{
		// 1my_debug("%s.%d: %s device was create with name = %s\n",fimx6d.name, pdev->id, __func__, fimx6d.f_dev[pdev->id].name);
	}

	if(!fimx6d.is_init)	{
    	fimx6d.is_init = 1;
    	// flexcan_proc_dir_init();
    }

    flexcan_net_open(pdev->id);

	return 0;

 err_device_create:
 	if(!fimx6d.is_init)	{
		class_destroy(fimx6d.f_class);
	}
 err_class_create:
 err_cdev_add:	
 	if(!fimx6d.is_init)	{
 		unregister_chrdev_region(fimx6d.f_dev[pdev->id].f_devt, DEV_COUNT);
 	}
 err_reg_chrdev:
	iounmap(base);
 failed_map:
	release_mem_region(mem->start, mem_size);
 failed_get:
	clk_put(clk);
 failed_clock:
	return err;
}

static int __devexit flexcan_remove(struct platform_device *pdev)
{
	struct resource *mem;

	// 1my_debug("%s.%d: %s remove flexcan driver start\n", fimx6d.name, pdev->id, __func__);

	flexcan_net_close(pdev->id);

	// flexcan_proc_file_exit(pdev->id);
	// 1my_debug("%s.%d: %s device destroy\n", fimx6d.name, pdev->id, __func__);
	device_destroy(fimx6d.f_class, fimx6d.f_dev[pdev->id].f_devt);
	if(!fimx6d.is_init)	{
		flexcan_proc_dir_exit();
		class_destroy(fimx6d.f_class);
	}

	// 1my_debug("%s.%d: %s cdev del\n", fimx6d.name, pdev->id, __func__);
	cdev_del(&fimx6d.f_dev[pdev->id].f_cdev);
	if(!fimx6d.is_init)	{
		unregister_chrdev_region(fimx6d.f_dev[pdev->id].f_devt, DEV_COUNT);

		/* нужно перенести, но куда?..... */
		// flexcan_proc_dir_exit();
	}

	if(fimx6d.is_init)	{
    	fimx6d.is_init = 0;
    }

	platform_set_drvdata(pdev, NULL);
	// 1my_debug("%s.%d: %s iounmap\n", fimx6d.name, pdev->id, __func__);
	iounmap(fimx6d.f_dev[pdev->id].f_base);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	// 1my_debug("%s.%d: %s release mem region\n", fimx6d.name, pdev->id, __func__);
	release_mem_region(mem->start, resource_size(mem));

	clk_put(fimx6d.f_dev[pdev->id].clk);

	return 0;
}

#ifdef CONFIG_PM
static int flexcan_suspend(struct platform_device *pdev, pm_message_t state)
{
	int ret;

	fimx6d.f_dev[pdev->id].stats.state = CAN_STATE_SLEEPING;
	/* enable stop request for wakeup */
	if (fimx6d.version >= FLEXCAN_VER_10_0_12)
		mxc_iomux_set_gpr_register(13, 28, 1, 1);

	ret = irq_set_irq_wake(fimx6d.f_dev[pdev->id].irq_num, 1);
	if (ret)	{
		return ret;
	}

	return 0;
}

static int flexcan_resume(struct platform_device *pdev)
{
	int ret;

	// 1my_debug("%s.%d: %s irq set irq wake\n", fimx6d.name, pdev->id, __func__);
	ret = irq_set_irq_wake(fimx6d.f_dev[pdev->id].irq_num, 0);
	if (ret)	{
		return ret;
	}
	fimx6d.f_dev[pdev->id].stats.state = CAN_STATE_ERROR_ACTIVE;

	return 0;
}
#else
#define flexcan_suspend NULL
#define flexcan_resume NULL
#endif


static struct platform_driver flexcan_driver = {
	.driver.name = 	DRV_NAME,
	.probe = 		flexcan_probe,
	.id_table = 	flexcan_devtype,
	.remove = 		__devexit_p(flexcan_remove),
	.suspend = 		flexcan_suspend,
	.resume = 		flexcan_resume,
};

static int __init flexcan_init(void)
{
	int ret = 0;
	ret = platform_driver_register(&flexcan_driver);
	return ret;
}

static void __exit flexcan_exit(void)
{
	platform_driver_unregister(&flexcan_driver);
}

module_init(flexcan_init);
module_exit(flexcan_exit);

MODULE_AUTHOR("Sascha Hauer <kernel@pengutronix.de>, "
	      "Marc Kleine-Budde <kernel@pengutronix.de>, "
	      "Mikita Dzivakou <grommerin@gmail.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("CAN port driver (with file operations) for flexcan based chip");
