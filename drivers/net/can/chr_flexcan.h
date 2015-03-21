#ifndef FLEXCAN_H
#define FLEXCAN_H

#include <linux/ioctl.h>
#include <linux/types.h>

#include <linux/can.h>
#include <linux/can/netlink.h>
#include <linux/can/error.h>


#define CAN_BITRATE_1000	(0x01 << 8)		/* Set bitrate 1000 */
#define CAN_BITRATE_500		(0x02 << 8)		/* Set bitrate 500 */
#define CAN_BITRATE_250		(0x03 << 8)		/* Set bitrate 250 */
#define CAN_BITRATE_125		(0x04 << 8)		/* Set bitrate 125 */
#define CAN_BITRATE_100		(0x05 << 8)		/* Set bitrate 100 */
#define CAN_BITRATE_MASK	0x0000ff00	
#define DRIVER_DEBUG_ON		(0x40 << 8)	
#define DRIVER_DEBUG_OFF	(0x80 << 8)		

struct flexcan_mb {
	u32 can_ctrl;
	u32 can_id;
	u32 data[2];
};

struct flexcan_frame {
	struct flexcan_mb mb;
	struct timeval time;
};

struct flexcan_stats {
	u32 rx_frames;			// количество принятых фреймов
	u32 tx_frames;			// количество отправленных фреймов
	/* возможно стоит предусмотреть увеличение счетчиков байт данных т.к. может переполниться за 8 часов */
	u32 rx_bytes;				// количество принятых байт данных
	u32 tx_bytes;				// количество отправленных байт данных

	u32 state_req;			// количество запросов проверки состояния
	u32 frame_req;			// количество прерываний чтения фреймов
	u32 irq_num;				// количество прерываний

	u32 err_over;				// количество переполнений буффера
	u32 err_warn;				// количество заполнений буффера
	u32 err_frame;			// количество ошибочных фреймов
	u32 err_drop;				// количество потерянных при чтении фреймов
	u32 err_length;			// количество ошибок длины фреймов

	u32 reg_mcr;				// состояние регистра mcr
	u32 reg_ctrl;				// состояние регистра ctrl

	u32 freq;					// частота модуля

	enum can_state state;
	struct can_bittiming_const *bittiming_const;
};

#define FLEXCAN_IOC_MAGIC			0x81
//Отныне и навсегда за сим местом лежат новые команды для IOCTL запросов к драйверу flexcan. Да будет так.

//#define FLEXCAN_IOCTL_CMD 	_IO(FLEXCAN_IOC_MAGIC, 0x00)
#define FLEXCAN_IOCTL_READ 		_IOR(FLEXCAN_IOC_MAGIC, 0x00, int *)
#define FLEXCAN_IOCTL_WRITE 	_IOW(FLEXCAN_IOC_MAGIC, 0x01, int *)
#define FLEXCAN_IOCTL_WR_RD		_IOWR(FLEXCAN_IOC_MAGIC, 0x02, int *)

// struct strim_flexcan_settings {
	// unsigned int settings;		
	/* Макросы CAN_CTRLMODE описаны в include/linux/netlink.h
	 * Макросы CAN_BITRATE описаны выше в этом файле
	 *
	 */
// };

#endif
