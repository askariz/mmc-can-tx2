#ifndef _mmc_protocol_h
#define _mmc_protocol_h

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/input.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/uio.h>
#include <net/if.h>

#pragma pack(1)
typedef struct {
	uint32_t sof;
	uint8_t  ver;
	uint16_t id;
	uint8_t len;
	uint16_t channel[10];
	uint32_t crc;
}ctrl_info_t;
#pragma pack()

#pragma pack(1)
typedef struct {
	uint8_t head;
	uint8_t type;
	uint8_t len;
	uint16_t channel[8];
	uint8_t crc;
}ctrl_info_new_t;
#pragma pack()

void ctrl_payload_new(int sock , uint16_t *data);
void ctrl_payload_old(int sock ,uint16_t *buf , uint8_t size );
uint32_t CRC32Software_1( uint8_t *pData, uint16_t Length );
uint8_t CRC8Software(uint8_t *ptr, uint16_t len);
void try_parse_can_info(uint8_t* data , uint8_t size);

extern struct canfd_frame frame;
extern struct msghdr msg;
#endif
