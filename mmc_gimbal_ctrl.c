/*	Description	: protocol for mmc can bus gimbal
 *	Author		: ChrisRiz
 *	Date		: 2018-06-06
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
#include <errno.h>
#include <termios.h>
#include <unistd.h>

#include "terminal.h"
#include "lib.h"
#include "mmc_protocol.h"

#define DEBUG
#define MAXIFNAMES 30 /* size of receive name index to omit ioctls */
#define MAXCOL 6      /* number of different colors for colorized output */
#define ANYDEV "any"  /* name of interface to receive from any CAN interface */
#define ANL "\r\n"    /* newline in ASC mode */

int sock;
static char devname[MAXIFNAMES][IFNAMSIZ+1];
static int  dindex[MAXIFNAMES];
static int  max_devname_len; /* to prevent frazzled device name output */ 
const int canfd_on = 1;

struct cmsghdr *cmsg;
can_err_mask_t err_mask;
extern struct canfd_frame can_frame;
extern struct msghdr msg;
static volatile int running = 1;


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

int idx2dindex(int ifidx, int socket) {
	int i;
	struct ifreq ifr;

	for (i=0; i < MAXIFNAMES; i++) {
		if (dindex[i] == ifidx)
			return i;
	}

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

fd_set rdfs;

unsigned char down_causes_exit = 1;
unsigned char view = 0;
int opt, ret;
int  numfilter;
int join_filter;

struct sockaddr_can addr;
char ctrlmsg[CMSG_SPACE(sizeof(struct timeval) + 3*sizeof(struct timespec) + sizeof(__u32))];
struct iovec iov;
struct ifreq ifr;
struct timeval  *timeout_current = NULL;

/*	Description	: init function for can device
 *	Parameters 	: 1 for sucess and -1 for failed
 *	Author		: ChrisRiz
 *	Date		: 2018-06-06
 */
int init(){
	char *dev_name = "can0";
#ifdef DEBUG
	printf("open '%s'.\n",dev_name);
#endif
	sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if (sock < 0) {
		printf("socket init error \n");
		return -1;
	}

	max_devname_len = strlen(dev_name); /* no ',' found => no filter definitions */
	if (max_devname_len >= IFNAMSIZ) {
	    printf("name of CAN device '%s' is too long!\n", dev_name);
		return -1;
	}
	addr.can_family = AF_CAN;

	memset(&ifr.ifr_name, 0, sizeof(ifr.ifr_name));
	strncpy(ifr.ifr_name, dev_name, max_devname_len);

#ifdef DEBUG
	printf("using interface name '%s'.\n", ifr.ifr_name);
#endif

	if (strcmp(ANYDEV, ifr.ifr_name)) {
		if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0) {
			printf("SIOCGIFINDEX\n");
			return -1;
		}
		addr.can_ifindex = ifr.ifr_ifindex;
	} else
		addr.can_ifindex = 0; /* any can interface */

	/* try to switch the socket into CAN FD mode */
	setsockopt(sock, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &canfd_on, sizeof(canfd_on));

	if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		printf("bind error\n");
		return -1;
	}
	return 1;
}


int main(int argc, char **argv)
{
	if(!init()){
		printf("can init failed\n");
		return 0;
	}
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
		for(int i = 0 ; i <8 ; i++)
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

		FD_ZERO(&rdfs);
		FD_SET(sock, &rdfs);

		if ((ret = select(sock+1, &rdfs, NULL, NULL, timeout_current)) <= 0) {
			printf("select");
			running = 0;
			continue;
		}

		{  /* check all CAN RAW sockets */

			if (FD_ISSET(sock, &rdfs)) {
				int idx;
				/* these settings may be modified by recvmsg() */
				iov.iov_len = sizeof(frame);
				msg.msg_namelen = sizeof(addr);
				msg.msg_controllen = sizeof(ctrlmsg);
				msg.msg_flags = 0;

				int nbytes = recvmsg(sock, &msg, MSG_DONTWAIT);
				idx = idx2dindex(addr.can_ifindex, sock);

				if (nbytes < 0) {
					if ((errno == ENETDOWN) && !down_causes_exit) {
						fprintf(stderr, "%s: interface down\n", devname[idx]);
						continue;
					}
					perror("read");
					return 1;
				}
				int maxdlen;
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
				ctrl_payload_new(sock,channel_value);
				usleep(1000);
			}
		}
	}

	close(sock);

	return 0;
}
// 查看can口状态 ip -s -d link show can0
