/*
 *  MMC Gimbal Control Demo.
 *  Author: ChrisRiz
 *	Date  : 2018-03-21
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <ctype.h>
#include <libgen.h>
#include <time.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/uio.h>
#include <net/if.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/input.h>

#include "terminal.h"
#include "lib.h"
#define DEBUG

#define MAXSOCK 16    /* max. number of CAN interfaces given on the cmdline */
#define MAXIFNAMES 30 /* size of receive name index to omit ioctls */
#define MAXCOL 6      /* number of different colors for colorized output */
#define ANYDEV "any"  /* name of interface to receive from any CAN interface */
#define ANL "\r\n"    /* newline in ASC mode */

static char *cmdlinename;
static char devname[MAXIFNAMES][IFNAMSIZ+1];
static int  dindex[MAXIFNAMES];
static int  max_devname_len; /* to prevent frazzled device name output */ 
const int canfd_on = 1;

struct msghdr msg;
struct cmsghdr *cmsg;
struct can_filter *rfilter;
can_err_mask_t err_mask;
struct canfd_frame frame;

extern int optind;

static volatile int running = 1;

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

uint32_t Crc32Table_1[ 256 ] =
{
  0x00000000, 0x04C11DB7, 0x09823B6E, 0x0D4326D9, 0x130476DC, 0x17C56B6B,
  0x1A864DB2, 0x1E475005, 0x2608EDB8, 0x22C9F00F, 0x2F8AD6D6, 0x2B4BCB61,
  0x350C9B64, 0x31CD86D3, 0x3C8EA00A, 0x384FBDBD, 0x4C11DB70, 0x48D0C6C7,
  0x4593E01E, 0x4152FDA9, 0x5F15ADAC, 0x5BD4B01B, 0x569796C2, 0x52568B75,
  0x6A1936C8, 0x6ED82B7F, 0x639B0DA6, 0x675A1011, 0x791D4014, 0x7DDC5DA3,
  0x709F7B7A, 0x745E66CD, 0x9823B6E0, 0x9CE2AB57, 0x91A18D8E, 0x95609039,
  0x8B27C03C, 0x8FE6DD8B, 0x82A5FB52, 0x8664E6E5, 0xBE2B5B58, 0xBAEA46EF,
  0xB7A96036, 0xB3687D81, 0xAD2F2D84, 0xA9EE3033, 0xA4AD16EA, 0xA06C0B5D,
  0xD4326D90, 0xD0F37027, 0xDDB056FE, 0xD9714B49, 0xC7361B4C, 0xC3F706FB,
  0xCEB42022, 0xCA753D95, 0xF23A8028, 0xF6FB9D9F, 0xFBB8BB46, 0xFF79A6F1,
  0xE13EF6F4, 0xE5FFEB43, 0xE8BCCD9A, 0xEC7DD02D, 0x34867077, 0x30476DC0,
  0x3D044B19, 0x39C556AE, 0x278206AB, 0x23431B1C, 0x2E003DC5, 0x2AC12072,
  0x128E9DCF, 0x164F8078, 0x1B0CA6A1, 0x1FCDBB16, 0x018AEB13, 0x054BF6A4,
  0x0808D07D, 0x0CC9CDCA, 0x7897AB07, 0x7C56B6B0, 0x71159069, 0x75D48DDE,
  0x6B93DDDB, 0x6F52C06C, 0x6211E6B5, 0x66D0FB02, 0x5E9F46BF, 0x5A5E5B08,
  0x571D7DD1, 0x53DC6066, 0x4D9B3063, 0x495A2DD4, 0x44190B0D, 0x40D816BA,
  0xACA5C697, 0xA864DB20, 0xA527FDF9, 0xA1E6E04E, 0xBFA1B04B, 0xBB60ADFC,
  0xB6238B25, 0xB2E29692, 0x8AAD2B2F, 0x8E6C3698, 0x832F1041, 0x87EE0DF6,
  0x99A95DF3, 0x9D684044, 0x902B669D, 0x94EA7B2A, 0xE0B41DE7, 0xE4750050,
  0xE9362689, 0xEDF73B3E, 0xF3B06B3B, 0xF771768C, 0xFA325055, 0xFEF34DE2,
  0xC6BCF05F, 0xC27DEDE8, 0xCF3ECB31, 0xCBFFD686, 0xD5B88683, 0xD1799B34,
  0xDC3ABDED, 0xD8FBA05A, 0x690CE0EE, 0x6DCDFD59, 0x608EDB80, 0x644FC637,
  0x7A089632, 0x7EC98B85, 0x738AAD5C, 0x774BB0EB, 0x4F040D56, 0x4BC510E1,
  0x46863638, 0x42472B8F, 0x5C007B8A, 0x58C1663D, 0x558240E4, 0x51435D53,
  0x251D3B9E, 0x21DC2629, 0x2C9F00F0, 0x285E1D47, 0x36194D42, 0x32D850F5,
  0x3F9B762C, 0x3B5A6B9B, 0x0315D626, 0x07D4CB91, 0x0A97ED48, 0x0E56F0FF,
  0x1011A0FA, 0x14D0BD4D, 0x19939B94, 0x1D528623, 0xF12F560E, 0xF5EE4BB9,
  0xF8AD6D60, 0xFC6C70D7, 0xE22B20D2, 0xE6EA3D65, 0xEBA91BBC, 0xEF68060B,
  0xD727BBB6, 0xD3E6A601, 0xDEA580D8, 0xDA649D6F, 0xC423CD6A, 0xC0E2D0DD,
  0xCDA1F604, 0xC960EBB3, 0xBD3E8D7E, 0xB9FF90C9, 0xB4BCB610, 0xB07DABA7,
  0xAE3AFBA2, 0xAAFBE615, 0xA7B8C0CC, 0xA379DD7B, 0x9B3660C6, 0x9FF77D71,
  0x92B45BA8, 0x9675461F, 0x8832161A, 0x8CF30BAD, 0x81B02D74, 0x857130C3,
  0x5D8A9099, 0x594B8D2E, 0x5408ABF7, 0x50C9B640, 0x4E8EE645, 0x4A4FFBF2,
  0x470CDD2B, 0x43CDC09C, 0x7B827D21, 0x7F436096, 0x7200464F, 0x76C15BF8,
  0x68860BFD, 0x6C47164A, 0x61043093, 0x65C52D24, 0x119B4BE9, 0x155A565E,
  0x18197087, 0x1CD86D30, 0x029F3D35, 0x065E2082, 0x0B1D065B, 0x0FDC1BEC,
  0x3793A651, 0x3352BBE6, 0x3E119D3F, 0x3AD08088, 0x2497D08D, 0x2056CD3A,
  0x2D15EBE3, 0x29D4F654, 0xC5A92679, 0xC1683BCE, 0xCC2B1D17, 0xC8EA00A0,
  0xD6AD50A5, 0xD26C4D12, 0xDF2F6BCB, 0xDBEE767C, 0xE3A1CBC1, 0xE760D676,
  0xEA23F0AF, 0xEEE2ED18, 0xF0A5BD1D, 0xF464A0AA, 0xF9278673, 0xFDE69BC4,
  0x89B8FD09, 0x8D79E0BE, 0x803AC667, 0x84FBDBD0, 0x9ABC8BD5, 0x9E7D9662,
  0x933EB0BB, 0x97FFAD0C, 0xAFB010B1, 0xAB710D06, 0xA6322BDF, 0xA2F33668,
  0xBCB4666D, 0xB8757BDA, 0xB5365D03, 0xB1F740B4 };

//软件crc8校验
static const uint8_t crc8_table[] =
{
    0x00,0x31,0x62,0x53,0xc4,0xf5,0xa6,0x97,0xb9,0x88,0xdb,0xea,0x7d,0x4c,0x1f,0x2e,
    0x43,0x72,0x21,0x10,0x87,0xb6,0xe5,0xd4,0xfa,0xcb,0x98,0xa9,0x3e,0x0f,0x5c,0x6d,
    0x86,0xb7,0xe4,0xd5,0x42,0x73,0x20,0x11,0x3f,0x0e,0x5d,0x6c,0xfb,0xca,0x99,0xa8,
    0xc5,0xf4,0xa7,0x96,0x01,0x30,0x63,0x52,0x7c,0x4d,0x1e,0x2f,0xb8,0x89,0xda,0xeb,
    0x3d,0x0c,0x5f,0x6e,0xf9,0xc8,0x9b,0xaa,0x84,0xb5,0xe6,0xd7,0x40,0x71,0x22,0x13,
    0x7e,0x4f,0x1c,0x2d,0xba,0x8b,0xd8,0xe9,0xc7,0xf6,0xa5,0x94,0x03,0x32,0x61,0x50,
    0xbb,0x8a,0xd9,0xe8,0x7f,0x4e,0x1d,0x2c,0x02,0x33,0x60,0x51,0xc6,0xf7,0xa4,0x95,
    0xf8,0xc9,0x9a,0xab,0x3c,0x0d,0x5e,0x6f,0x41,0x70,0x23,0x12,0x85,0xb4,0xe7,0xd6,
    0x7a,0x4b,0x18,0x29,0xbe,0x8f,0xdc,0xed,0xc3,0xf2,0xa1,0x90,0x07,0x36,0x65,0x54,
    0x39,0x08,0x5b,0x6a,0xfd,0xcc,0x9f,0xae,0x80,0xb1,0xe2,0xd3,0x44,0x75,0x26,0x17,
    0xfc,0xcd,0x9e,0xaf,0x38,0x09,0x5a,0x6b,0x45,0x74,0x27,0x16,0x81,0xb0,0xe3,0xd2,
    0xbf,0x8e,0xdd,0xec,0x7b,0x4a,0x19,0x28,0x06,0x37,0x64,0x55,0xc2,0xf3,0xa0,0x91,
    0x47,0x76,0x25,0x14,0x83,0xb2,0xe1,0xd0,0xfe,0xcf,0x9c,0xad,0x3a,0x0b,0x58,0x69,
    0x04,0x35,0x66,0x57,0xc0,0xf1,0xa2,0x93,0xbd,0x8c,0xdf,0xee,0x79,0x48,0x1b,0x2a,
    0xc1,0xf0,0xa3,0x92,0x05,0x34,0x67,0x56,0x78,0x49,0x1a,0x2b,0xbc,0x8d,0xde,0xef,
    0x82,0xb3,0xe0,0xd1,0x46,0x77,0x24,0x15,0x3b,0x0a,0x59,0x68,0xff,0xce,0x9d,0xac
};
//------------------------------------------------------------------------------
//--函数名：  cal_crc_table
//--函数功能：软件cr校验
//--输入参数： ptr，len
//--输出参数： crc
//------------------------------------------------------------------------------
uint8_t CRC8Software(uint8_t *ptr, uint16_t len)
{
	uint8_t  crc = 0x00;
    while (len--){
        crc = crc8_table[crc ^ *ptr++];
    }
    return (crc);
}

//查表法
uint32_t CRC32Software_1( uint8_t *pData, uint16_t Length )
{
	uint32_t nReg; //CRC寄存器
	uint32_t nTemp = 0;
	uint16_t i, n;
	nReg = 0xFFFFFFFF;
	for ( n = 0; n < Length ; n++ ){
		nReg ^= (uint32_t) pData[ 4*n + 3  ] << 24 | pData[ 4*n + 2  ] << 16 |pData[ 4*n + 1  ] << 8 | pData[ 4*n  ]  ;
		for ( i = 0; i < 4; i++ ){
			nTemp = Crc32Table_1[ ( uint8_t )( ( nReg >> 24 ) & 0xff ) ]; //取一个字节，查表
			nReg <<= 8; //丢掉计算过的头一个BYTE
			nReg ^= nTemp; //与前一个BYTE的计算结果异或
		}
	}
	return nReg;
}

void ctrl_payload_old(int s ,uint16_t *buf , uint8_t size ){
	ctrl_info_t ctrl_info;
	ctrl_info.sof = 0x5AA55AA5;
	ctrl_info.ver = 0x10;
	ctrl_info.id = 0x6666;
	ctrl_info.len = sizeof(ctrl_info) -12;
	memcpy(ctrl_info.channel,buf,10);
	ctrl_info.crc = CRC32Software_1((uint8_t*) ctrl_info.channel,5);

	// 此处直接发送控制指令
	int len = 0 , t;
	uint8_t *pdata = (uint8_t*)&ctrl_info;
	uint8_t sendbuf[100];
	for(t = 0 ; t < sizeof(ctrl_info) ; t++)
		sendbuf[t] = *pdata++;

	frame.can_id = 0x123;
	while(len < sizeof(ctrl_info)){
		if( sizeof(ctrl_info) - len >= 8){
			memcpy(frame.data,&sendbuf[len],8);
			int nbytes = sendmsg(s, &msg, MSG_DONTWAIT);
			if(nbytes > 0){
//				printf("---->send msg %d \n",8);
				len += 8;
			}
		}else{
			memcpy(frame.data,&sendbuf[len], sizeof(ctrl_info) - 8);
			int nbytes = sendmsg(s, &msg, MSG_DONTWAIT);
			if(nbytes > 0){
//				printf("---->send msg %d \n",sizeof(ctrl_info) - len);
				len += sizeof(ctrl_info) - len;
			}
		}
	}
}

void ctrl_payload_new(int s , uint16_t *data){
	int len = 0 , t;
	ctrl_info_new_t ctrl_info;
	uint8_t sendbuf[100];

	ctrl_info.head = 0xa5;
	ctrl_info.type =0x03;
	ctrl_info.len = 18;

	memcpy(&ctrl_info.channel,data,sizeof(ctrl_info.channel));
	ctrl_info.crc = CRC8Software(&ctrl_info.type,ctrl_info.len);

	uint8_t *pdata = (uint8_t*)&ctrl_info;

	for(t = 0 ; t < sizeof(ctrl_info) ; t++)
		sendbuf[t] = *pdata++;

	frame.can_id = 0x8b;
	while(len < sizeof(ctrl_info)){
		if( sizeof(ctrl_info) - len >= 8){
			memcpy(frame.data,&sendbuf[len],8);
			frame.len = 8;
			int nbytes = sendmsg(s, &msg, MSG_DONTWAIT);
			if(nbytes > 0){
				len += 8;
			}
		}else{
			memcpy(frame.data,&sendbuf[len], sizeof(ctrl_info) - len);
			frame.len = sizeof(ctrl_info) - len;
			int nbytes = sendmsg(s, &msg, MSG_DONTWAIT);
			if(nbytes > 0){
				len += sizeof(ctrl_info) - len;
			}
		}
	}
}

uint8_t parse_buf[101];
int buf_len = 0;
void try_parse_can_info(uint8_t* data , uint8_t size){
	memcpy(&parse_buf[buf_len],data,size);
	buf_len += size;
	if(parse_buf[0] == 0xa5){
		if(buf_len >= parse_buf[2] + 2 || buf_len > 100){
			buf_len = 0;
			if(CRC8Software(&parse_buf[1],parse_buf[2]) == parse_buf[parse_buf[2] + 1]){
				//  此处解析出一个can 包
#ifdef DEBUG
				printf("succes parse can info %02X %02X \n",parse_buf[0],parse_buf[1]);
#endif
			}
		}
	}else buf_len = 0;
}

void print_usage(char *prg)
{
	fprintf(stderr, "\nUsage: %s [options] <CAN interface>+\n", prg);
	fprintf(stderr, "  (use CTRL-C to terminate %s)\n\n", prg);
	fprintf(stderr, "Key function: \n");
	fprintf(stderr, "	w:          control the pitch 	up		of the gimbal\n");
	fprintf(stderr, "   s:          control the pitch 	down	of the gimbal\n");
	fprintf(stderr, "   a:          control the yaw		left  	of the gimbal\n");
	fprintf(stderr, "   d:          control the yaw  	right 	of the gimbal\n");
	fprintf(stderr, "   z:          control the zoom 	in 		of the gimbal\n");
	fprintf(stderr, "   x:          control the zomm 	out 	of the gimbal\n");
	fprintf(stderr, "   q:         	exit\n");
	fprintf(stderr, "===Author:MMC ChrisRiz====\n");
}

int idx2dindex(int ifidx, int socket) {

	int i;
	struct ifreq ifr;

	for (i=0; i < MAXIFNAMES; i++) {
		if (dindex[i] == ifidx)
			return i;
	}

	/* create new interface index cache entry */

	/* remove index cache zombies first */
	for (i=0; i < MAXIFNAMES; i++) {
		if (dindex[i]) {
			ifr.ifr_ifindex = dindex[i];
			if (ioctl(socket, SIOCGIFNAME, &ifr) < 0)
				dindex[i] = 0;
		}
	}

	for (i=0; i < MAXIFNAMES; i++)
		if (!dindex[i]) /* free entry */
			break;

	if (i == MAXIFNAMES) {
		fprintf(stderr, "Interface index cache only supports %d interfaces.\n",
		       MAXIFNAMES);
		exit(1);
	}

	dindex[i] = ifidx;

	ifr.ifr_ifindex = ifidx;
	if (ioctl(socket, SIOCGIFNAME, &ifr) < 0)
		perror("SIOCGIFNAME");

	if (max_devname_len < strlen(ifr.ifr_name))
		max_devname_len = strlen(ifr.ifr_name);

	strcpy(devname[i], ifr.ifr_name);

#ifdef DEBUG
	printf("new index %d (%s)\n", i, devname[i]);
#endif

	return i;
}


int main(int argc, char **argv)
{
	fd_set rdfs;
	int s;
	unsigned char down_causes_exit = 1;
	unsigned char view = 0;
	int rcvbuf_size = 0;
	int opt, ret;
	int  numfilter;
	int join_filter;
	char *ptr, *nptr;
	struct sockaddr_can addr;
	char ctrlmsg[CMSG_SPACE(sizeof(struct timeval) + 3*sizeof(struct timespec) + sizeof(__u32))];
	struct iovec iov;
	int nbytes, i, maxdlen;
	struct ifreq ifr;
	struct timeval  *timeout_current = NULL;

	print_usage(basename(argv[0]));

	while ((opt = getopt(argc, argv, "t:HciaSs:b:B:u:lDdxLn:r:heT:?")) != -1) {
		switch (opt) {
		case 't':
			break;

		default:
			print_usage(basename(argv[0]));
			exit(1);
			break;
		}
	}

	if (optind == argc) {
		print_usage(basename(argv[0]));
		exit(0);
	}
	{
		ptr = "can0";
		nptr = strchr(ptr, ',');

#ifdef DEBUG
		printf("open %d '%s'.\n", i, ptr);
#endif

		s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
		if (s < 0) {
			perror("socket");
			return 1;
		}

		cmdlinename = ptr; /* save pointer to cmdline name of this socket */

		if (nptr)
			nbytes = nptr - ptr;  /* interface name is up the first ',' */
		else
			nbytes = strlen(ptr); /* no ',' found => no filter definitions */

		if (nbytes >= IFNAMSIZ) {
			fprintf(stderr, "name of CAN device '%s' is too long!\n", ptr);
			return 1;
		}

		if (nbytes > max_devname_len)
			max_devname_len = nbytes; /* for nice printing */

		addr.can_family = AF_CAN;

		memset(&ifr.ifr_name, 0, sizeof(ifr.ifr_name));
		strncpy(ifr.ifr_name, ptr, nbytes);

#ifdef DEBUG
		printf("using interface name '%s'.\n", ifr.ifr_name);
#endif

		if (strcmp(ANYDEV, ifr.ifr_name)) {
			if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
				perror("SIOCGIFINDEX");
				exit(1);
			}
			addr.can_ifindex = ifr.ifr_ifindex;
		} else
			addr.can_ifindex = 0; /* any can interface */

		if (nptr) {

			/* found a ',' after the interface name => check for filters */

			/* determine number of filters to alloc the filter space */
			numfilter = 0;
			ptr = nptr;
			while (ptr) {
				numfilter++;
				ptr++; /* hop behind the ',' */
				ptr = strchr(ptr, ','); /* exit condition */
			}

			rfilter = malloc(sizeof(struct can_filter) * numfilter);
			if (!rfilter) {
				fprintf(stderr, "Failed to create filter space!\n");
				return 1;
			}

			numfilter = 0;
			err_mask = 0;
			join_filter = 0;

			while (nptr) {

				ptr = nptr+1; /* hop behind the ',' */
				nptr = strchr(ptr, ','); /* update exit condition */

				if (sscanf(ptr, "%x:%x",
					   &rfilter[numfilter].can_id, 
					   &rfilter[numfilter].can_mask) == 2) {
 					rfilter[numfilter].can_mask &= ~CAN_ERR_FLAG;
					numfilter++;
				} else if (sscanf(ptr, "%x~%x",
						  &rfilter[numfilter].can_id, 
						  &rfilter[numfilter].can_mask) == 2) {
 					rfilter[numfilter].can_id |= CAN_INV_FILTER;
 					rfilter[numfilter].can_mask &= ~CAN_ERR_FLAG;
					numfilter++;
				} else if (*ptr == 'j' || *ptr == 'J') {
					join_filter = 1;
				} else if (sscanf(ptr, "#%x", &err_mask) != 1) { 
					fprintf(stderr, "Error in filter option parsing: '%s'\n", ptr);
					return 1;
				}
			}

			if (err_mask)
				setsockopt(s, SOL_CAN_RAW, CAN_RAW_ERR_FILTER,
					   &err_mask, sizeof(err_mask));

			if (join_filter && setsockopt(s, SOL_CAN_RAW, CAN_RAW_JOIN_FILTERS,
						      &join_filter, sizeof(join_filter)) < 0) {
				perror("setsockopt CAN_RAW_JOIN_FILTERS not supported by your Linux Kernel");
				return 1;
			}

			if (numfilter)
				setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER,
					   rfilter, numfilter * sizeof(struct can_filter));

			free(rfilter);

		} /* if (nptr) */

		/* try to switch the socket into CAN FD mode */
		setsockopt(s, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &canfd_on, sizeof(canfd_on));

		if (rcvbuf_size) {

			int curr_rcvbuf_size;
			socklen_t curr_rcvbuf_size_len = sizeof(curr_rcvbuf_size);

			/* try SO_RCVBUFFORCE first, if we run with CAP_NET_ADMIN */
			if (setsockopt(s, SOL_SOCKET, SO_RCVBUFFORCE,
				       &rcvbuf_size, sizeof(rcvbuf_size)) < 0) {
#ifdef DEBUG
				printf("SO_RCVBUFFORCE failed so try SO_RCVBUF ...\n");
#endif
				if (setsockopt(s, SOL_SOCKET, SO_RCVBUF,
					       &rcvbuf_size, sizeof(rcvbuf_size)) < 0) {
					perror("setsockopt SO_RCVBUF");
					return 1;
				}

				if (getsockopt(s, SOL_SOCKET, SO_RCVBUF,
					       &curr_rcvbuf_size, &curr_rcvbuf_size_len) < 0) {
					perror("getsockopt SO_RCVBUF");
					return 1;
				}

				/* Only print a warning the first time we detect the adjustment */
				/* n.b.: The wanted size is doubled in Linux in net/sore/sock.c */
				if (!i && curr_rcvbuf_size < rcvbuf_size*2)
					fprintf(stderr, "The socket receive buffer size was "
						"adjusted due to /proc/sys/net/core/rmem_max.\n");
			}
		}

		if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
			perror("bind");
			return 1;
		}
	}

//	// 初始化按键
//	int keys_fd = open("/dev/input/event2",O_RDONLY|O_NONBLOCK);
//	if(keys_fd <= 0){
//		printf("open /dev/input/event2 error \n");
//		return 0;
//	}

	//打开控制终端
	int tty = open("/dev/tty", O_RDONLY | O_NONBLOCK);
	struct termios newt, oldt;
	tcgetattr(tty, &oldt);		//获取终端属性
	newt = oldt;
	newt.c_lflag &= ~( ICANON | ECHO );	//设置字符不缓冲且不回显
	tcsetattr(tty, TCSANOW, &newt);

	// 指定iov为frame的地址，再制定msg_iov地址为iov，这样直接就可以对frame赋值
	iov.iov_base = &frame;
	msg.msg_name = &addr;
	msg.msg_iov = &iov;
	msg.msg_iovlen = 1;
	msg.msg_control = &ctrlmsg;

	while (running) {
		// 获取终端输入字符
		//memset(channel_value,1500,sizeof(channel_value));
		char ch = "";
		read(tty, &ch, 1);
		uint16_t channel_value[10];
		for(i = 0 ; i <8 ; i++)
			channel_value[i] = 1500;

		switch(ch){
		case 'q'://还原终端属性
				tcsetattr(tty, TCSANOW, &oldt);
				fprintf(stderr, "Quit\n", ch);
				return 0;
			break;
		case 'w':// 云台上
			channel_value[2] = 2000;
			break;
		case 's':// 云台下
			channel_value[2] = 1000;
			break;
		case 'a':// 云台左
			channel_value[3] = 2000;

			break;
		case 'd':// 云台右
			channel_value[3] = 1000;
			break;
		case 'z':// 缩小
			channel_value[0] = 1000;

			break;
		case 'x':// 放大
			channel_value[0] = 2000;

			break;
		default:
			break;
		}
//
//		struct input_event event;
//		// 获取按键事件
//		if(read(keys_fd,&event,sizeof(event)) == sizeof(event)){
//			switch(event.code){
//			case KEY_Q:
//				printf("key event key Q\n");
//				break;
//			default:
//				break;
//			}
//		}

		FD_ZERO(&rdfs);

			FD_SET(s, &rdfs);

		if ((ret = select(s+1, &rdfs, NULL, NULL, timeout_current)) <= 0) {
			printf("select");
			running = 0;
			continue;
		}

		{  /* check all CAN RAW sockets */

			if (FD_ISSET(s, &rdfs)) {
				int idx;
				/* these settings may be modified by recvmsg() */
				iov.iov_len = sizeof(frame);
				msg.msg_namelen = sizeof(addr);
				msg.msg_controllen = sizeof(ctrlmsg);
				msg.msg_flags = 0;

				nbytes = recvmsg(s, &msg, MSG_DONTWAIT);
				idx = idx2dindex(addr.can_ifindex, s);

				if (nbytes < 0) {
					if ((errno == ENETDOWN) && !down_causes_exit) {
						fprintf(stderr, "%s: interface down\n", devname[idx]);
						continue;
					}
					perror("read");
					return 1;
				}

				if ((size_t)nbytes == CAN_MTU)
					maxdlen = CAN_MAX_DLEN;
				else if ((size_t)nbytes == CANFD_MTU)
					maxdlen = CANFD_MAX_DLEN;
				else {
					fprintf(stderr, "read: incomplete CAN frame\n");
					return 1;
				}

				/* once we detected a EFF frame indent SFF frames accordingly */
				if (frame.can_id & CAN_EFF_FLAG)
					view |= CANLIB_VIEW_INDENT_SFF;
#ifdef DEBUG
				printf("%*s", max_devname_len, devname[idx]);
				fprint_long_canframe(stdout, &frame, NULL, view, maxdlen);
				printf("\n");
#endif
				// 此处来解析can包,获取变焦倍数
				try_parse_can_info(frame.data,frame.len);

				 // 旧挂载的调试
//				ctrl_payload_old(s,data,10);
				ctrl_payload_new(s,channel_value);
				usleep(1000);
			}
		}
	}


		close(s);

	return 0;
}
// 查看can口状态 ip -s -d link show can0
