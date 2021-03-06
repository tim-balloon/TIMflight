/* -----------------------------------------------------------------------
 * -------------------------- TCP Communications -------------------------
 * -----------------------------------------------------------------------
 * This program is distributed under the GNU General Public License (GPL).
 *
 * The following was developed in the QNX Momentics IDE and compiled on
 * using QNX RTOS version 6.5.
 *
 * ------------------------- Description -------------------------------
 * This is TCP network setup and configuration for communications with
 * ground station links. Contained are the functions used to transfer
 * frame files over the network along with the format files and
 * linklists required to parse the received data. The main programs that
 * use this code are mole and bittlm.
 *
 * ---------------------------- Author ---------------------------------
 * L. Javier Romualdez (B.Eng Aerospace)
 * Institute for Aerospace Studies (UTIAS)
 * University of Toronto
 *
 * Created: August 29, 2017
 *
 * Copyright 2017 Javier Romualdez
 *
 * -------------------------- Revisions --------------------------------
 *
 *
 */
#ifndef TCPCONN_H_
#define TCPCONN_H_

#define SERVER_LL_REQ 0xEE000001
#define SERVER_LL_LIST_REQ 0xEE000002
#define SERVER_ARCHIVE_REQ 0xEE000003
#define SERVER_SERIAL_REQ 0xEE000004
#define SERVER_SET_LL_NAME_REQ 0xEE000005
#define SERVER_ARCHIVE_LIST_REQ 0xEE000006
#define SERVER_LL_NAME_REQ 0xEE000007

#define TCPCONN_LOOP 0x01
#define TCPCONN_FILE_RAW 0x02
#define TCPCONN_NOLOOP 0x04
#define TCPCONN_CLIENT_INIT 0x08
#define TCPCONN_RESOLVE_NAME 0x10
#define TCPCONN_FILE_RESET 0x20
#define TCPCONN_NO_DATA 0x40

#define TCP_PACKET_HEADER_SIZE 12
#define CLIENT_TELEM_PORT 40204

#ifndef MSG_MORE
#define MSG_MORE 0
#endif

#ifdef __cplusplus
extern "C" {
#endif

struct TCPCONN
{
  char ip[128];
  int fd;
	int flag;
	uint16_t theday, themonth, theyear;
	uint32_t serial;
};
  
typedef struct TCPCONN linklist_tcpconn_t;

#ifndef MAX
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#endif

void readTCPHeader(uint8_t *, uint32_t **, uint32_t **, uint16_t **, uint16_t **);

int connect_tcp(struct TCPCONN * );
int close_connection(struct TCPCONN *);
int64_t initialize_client_connection(struct TCPCONN * , uint32_t );
void set_linklist_server_port(unsigned int );
void set_linklist_client_port(unsigned int );

int request_server_list(struct TCPCONN * , char [][LINKLIST_SHORT_FILENAME_SIZE]);
int request_server_archive_list(struct TCPCONN * , char [][LINKLIST_SHORT_FILENAME_SIZE]);

int retrieve_data(struct TCPCONN * , uint8_t *, unsigned int);
int request_data(struct TCPCONN *, unsigned int, uint16_t *);

void set_server_linklist_name(struct TCPCONN * , char *);
void request_server_linklist_name(struct TCPCONN * , char *, unsigned int, unsigned int);
uint32_t request_server_file(struct TCPCONN * , char * , unsigned int );
int send_client_file(struct TCPCONN * , char * , uint32_t );

void send_client_error(struct TCPCONN *);
void linklist_server(void *);

void user_file_select(linklist_tcpconn_t *, char *);
uint32_t sync_with_server(struct TCPCONN *, char *, char *, unsigned int, superframe_t **, linklist_t **);

#ifdef __cplusplus
}
#endif

#endif /* TCPCONN_H_ */

