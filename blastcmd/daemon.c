/* daemon: groundstation BLAST command daemon
 *
 * This software is copyright (C) 2005 University of Toronto
 * 
 * This file is part of blastcmd.
 * 
 * blastcmd is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * blastcmd is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with blastcmd; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>

#include <errno.h>
#include <netdb.h>
#include <signal.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <ctype.h>

#include <sys/socket.h>
#include <sys/syslog.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/select.h>

#include <netinet/in.h>

#include <arpa/inet.h>

#include "netcmd.h"
#include "daemon.h"
#include "config.h"
#include "command_list.h"
#include "elog.h"

void setDefault(char *cmd, double D); // in cmdlist.c
double getDefault(char *cmd_in); // in cmdlist.c

#ifdef USE_AUTHENTICATION
/* read authorized IP addresses from file, and store in list */
#define AUTH_FILE DATA_ETC_DIR "/blastcmd.auth"
struct auth_addr {
  struct in_addr addr;
  uint32_t mask;
  struct auth_addr* next;
};

static char *pilot_host[2] = { "192.168.1.3:41414", "192.168.1.4:41414" };

#define BUF_SIZE (30 * 1024)

static struct auth_addr* auth_list = NULL;
static int auth_all_allowed = 0;

static int ReadAuth()
{
  FILE* stream;
  char buffer[BUF_SIZE];
  char* ip;
  char* tmp;
  struct auth_addr* new_addr;
  int n_auth = 0;

  /* disable all_allowed until it's encountered in the auth file */
  auth_all_allowed = 0;

  /* read config file */
  printf("Opening IP authorization file: \"%s\"\n", AUTH_FILE);
  if ((stream = fopen(AUTH_FILE, "r")) == NULL) {
    sprintf(buffer, "unable to open IP authorization file \"%s\"", AUTH_FILE);
    perror(buffer);
    exit(4);
  }

  while (fgets(buffer, BUF_SIZE, stream)) {
    /* remove comments */
    if ((tmp = strchr(buffer, '#')) != NULL)
      *tmp = '\0';

    /* strip newline */
    if ((tmp = strchr(buffer, '\n')) != NULL)
      *tmp = '\0';

    /* strip leading whitespace */
    ip = buffer + strspn(buffer, " \t");

    /* strip trailing whitespace */
    if ((tmp = strchr(ip, ' ')) != NULL)
      *tmp = '\0';
    if ((tmp = strchr(ip, '\t')) != NULL)
      *tmp = '\0';

    /* empty line */
    if (ip[0] == '\0')
      continue;

    /* check for magical "accept all" word */
    if (strncmp(ip, "INADDR_ANY", 10) == 0) {
      auth_all_allowed = 1;
      fclose(stream);
      /*printf("Authentication allowed for all hosts\n");*/
      return 0;
    }

    /* add to list */
    new_addr = malloc(sizeof(*new_addr));
    new_addr->next = auth_list;

    /* deal with CIDR notation -- IP addresses are stored in network order (big
     * endian, which is why these masks look backwards */
    if ((tmp = strchr(ip, '/')) != NULL) {
      *tmp = '\0';
      int prefix = atoi(tmp + 1);
      if (prefix == 31) /* two hosts */
        new_addr->mask = 0xEFFFFFFF;
      else if (prefix == 30) /* four hosts */
        new_addr->mask = 0xCFFFFFFF;
      else if (prefix == 29) /* eight hosts */
        new_addr->mask = 0x8FFFFFFF;
      else if (prefix == 28) /* sixteen hosts */
        new_addr->mask = 0xFFFFFFF;
      else if (prefix == 27) /* thirty-two hosts */
        new_addr->mask = 0xEFFFFFF;
      else if (prefix == 26) /* sixty-four hosts */
        new_addr->mask = 0xCFFFFFF;
      else if (prefix == 25) /* 128 hosts */
        new_addr->mask = 0x8FFFFFF;
      else if (prefix == 24) /* Class C network  */
        new_addr->mask = 0xFFFFFF;
      else if (prefix == 23) /* Class C x2 */
        new_addr->mask = 0xEFFFFF;
      else if (prefix == 22) /* Class C x4 */
        new_addr->mask = 0xCFFFFF;
      else if (prefix == 21) /* Class C x8 */
        new_addr->mask = 0x8FFFFF;
      else if (prefix == 20) /* Class C x16 */
        new_addr->mask = 0xFFFFF;
      else if (prefix == 19) /* Class C x32 */
        new_addr->mask = 0xEFFFF;
      else if (prefix == 18) /* Class C x64 */
        new_addr->mask = 0xCFFFF;
      else if (prefix == 17) /* Class C x128 */
        new_addr->mask = 0x8FFFF;
      else if (prefix == 16) /* Class B network */
        new_addr->mask = 0xFFFF;
      else if (prefix == 15) /* Class B x2 */
        new_addr->mask = 0xEFFF;
      else if (prefix == 14) /* Class B x4 */
        new_addr->mask = 0xCFFF;
      else if (prefix == 13) /* Class B x8 */
        new_addr->mask = 0x8FFF;
      else if (prefix == 12) /* Class B x16 */
        new_addr->mask = 0xFFF;
      else if (prefix == 11) /* Class B x32 */
        new_addr->mask = 0xEFF;
      else if (prefix == 10) /* Class B x64 */
        new_addr->mask = 0xCFF;
      else if (prefix == 9)  /* Class B x128 */
        new_addr->mask = 0x8FF;
      else if (prefix == 8)  /* Class A network */
        new_addr->mask = 0xFF;
      else                  /* for everything else, just consider the host */
        new_addr->mask = 0xFFFFFFFF;
    } else
      new_addr->mask = 0xFFFFFFFF;

    if (!inet_aton(ip, &new_addr->addr)) {
      /*printf("*WARNING* skipping malformed IP: \"%s\"\n", ip);*/
      free(new_addr);
    } else {
      auth_list = new_addr;
      n_auth++;
      /*printf("Authentication allowed for: %s\n", ip);*/
    }
  }
  /*printf("Allowing authentication from %d hosts\n", n_auth);*/
  fclose(stream);
  return n_auth;
}

static void ClearAuth()
{
  struct auth_addr* next;
  struct auth_addr* list = auth_list;
  while (list) {
    next = list->next;
    free(list);
    list = next;
  }
  auth_list = NULL;
}

static int IsAuth(struct in_addr* addr)
{
  struct auth_addr* list;

  /* read file every time authentication is checked */
  ClearAuth();
  ReadAuth();
  list = auth_list;

  /* do the check */
  if (auth_all_allowed)
    return 1;

  while (list) {
    if ((addr->s_addr & list->mask) == (list->addr.s_addr & list->mask))
      return 1; /* validated */
    list = list->next;
  }
  return 0;
}
#endif	/* USE_AUTHENTICATION */

int SIPRoute(int sock, int t_link, int t_route, char* buffer)
{
  int i_cmd;
  unsigned int i_ack;
  int count = 0;
  char* token[1024];
  char* ptr = buffer;

  /* reset error message */
  err_message[0] = 0;

  token[count++] = buffer;

  for (ptr = buffer + 1; *ptr != '\0'; ++ptr) {
    if (*ptr == ' ')
      *ptr = 0;
    else if (*(ptr - 1) == '\0')
      token[count++] = ptr;
  }
  
  /* Look for single packet commands */
  for (i_cmd = 0; i_cmd < N_SCOMMANDS; i_cmd++) {
    if (strncmp(token[0], scommands[i_cmd].name, SIZE_NAME) == 0) {
      SendScommand(sock, i_cmd, t_link, t_route, &i_ack);
      token[0] -= 3;

      if (i_ack == 0x0a)
        i_ack = 4;
      else if (i_ack == 0x0b)
        i_ack = 5;
      else if (i_ack == 0x0c)
        i_ack = 6;
      else if (i_ack == 0x0d)
        i_ack = 7;
      else if (i_ack == 0x0e)
        i_ack = 8;
      else if (i_ack == 0x0f)
        i_ack = 9;
      else if (i_ack == 0x10)
        i_ack = 13;
      else if (i_ack == 0x112)
        i_ack = 12;
      else
        i_ack = 0;

      WriteLogFile(0, NULL, i_ack);

      return i_ack;
    }
  }

  /* Look for multi packet commands */
  for (i_cmd = 0; i_cmd < N_MCOMMANDS; i_cmd++) {
    if (strncmp(token[0], mcommands[i_cmd].name, SIZE_NAME) == 0) {
      SendMcommand(sock, i_cmd, t_link, t_route, token + 1 , count - 1, &i_ack);
      token[0] -= 3;

      if (i_ack == 0x0a)
        i_ack = 4;
      else if (i_ack == 0x0b)
        i_ack =  5;
      else if (i_ack == 0x0c)
        i_ack =  6;
      else if (i_ack == 0x0d)
        i_ack =  7;
      else if (i_ack == 0x0e)
        i_ack =  8;
      else if (i_ack == 0x0f)
        i_ack =  9;
      else if (i_ack == 0x10)
        i_ack =  13;
      else if (i_ack == 0x102)
        i_ack =  2;
      else if (i_ack == 0x103)
        i_ack =  3;
      else if (i_ack == 0x110)
        i_ack =  10;
      else if (i_ack == 0x111)
        i_ack =  11;
      else if (i_ack == 0x112)
        i_ack =  12;
      else if (i_ack == 0x113)
        i_ack =  17;
      else
        i_ack = 0;

      WriteLogFile(0, NULL, i_ack);

      return i_ack;
    }
  }

  printf("Unknown command (%s) from socket %i\n", token[0], sock);
  return 1;
}

int ForwardRoute(char *buffer)
{
  if (pilot_host[1] == NULL) /* No host forward defined */
    return 18;

  if (NetCmdConnect(pilot_host[buffer[1] - '1'], 1, 0))
    return 18;

  /* lowercase ensures the downstream blastcmd won't elog it */
  buffer[0] = 'l';
  buffer[1] = '1';
  strcat(buffer, "\r\n");

  if (NetCmdTakeConn(1) == 0)
    return 18;

  int ack = NetCmdSendAndReceive(buffer, 1, 0, NULL);
  printf("Forwarded to %s\n", pilot_host[buffer[1]-'1']);

  WriteLogFile(0, NULL, ack);

  NetCmdDrop();
  return ack;
}

int SimpleRoute(int sock, int fd, char* buffer)
{
  if (write(fd, buffer, strlen(buffer)) < 0)
    perror("SimpleRoute write failed");
  if (write(fd, "\n", 1) < 0)
    perror("SimpleRoute write failed");
  return 0;
}

void StoreDefaultParameters(char *buffer) {
  char B[SIZE_NAME + MAX_N_PARAMS*(SIZE_PARNAME+2)];
  char *saveptr;
  char *cmd;
  char *param[MAX_N_PARAMS];
  int n_param=0;
  int i_cmd;
  int i_param;
  char cmdstr[SIZE_CMDPARNAME];
  int index_serial = 0;

  strncpy(B, buffer, SIZE_NAME + MAX_N_PARAMS*(SIZE_PARNAME+2)-1);

  cmd = strtok_r(B, " ", &saveptr);

  while ((param[n_param] = strtok_r(NULL, " " , &saveptr))!=NULL) {
    n_param++;
  }

  if (n_param>0) { // might be a multi-command
    for (i_cmd = 0; i_cmd < N_MCOMMANDS; i_cmd++) {
      if (strncmp(cmd, mcommands[i_cmd].name, SIZE_NAME) == 0) {
        if (mcommands[i_cmd].numparams == n_param) {
          index_serial = 0;
          for (i_param = 0; i_param<n_param; i_param++) {
            // FIXME: index parameter
            if (index_serial) {
              sprintf(cmdstr, "%s;%d;%s", cmd, index_serial,
                  mcommands[i_cmd].params[i_param].name);
            } else {
              sprintf(cmdstr, "%s;%s", cmd,
                  mcommands[i_cmd].params[i_param].name);
            }
            setDefault(cmdstr, atof(param[i_param]));
            if (mcommands[i_cmd].params[i_param].index_serial>0) {
              index_serial = atoi(param[i_param]);
            }
          }
        }
      }
    }
  }
}

int ExecuteCommand(int sock, int fd, int route, char* buffer, const char *user,
    int lastsock)
{
  int t_link = LINK_DEFAULT;
  int t_route = ROUTING_DEFAULT;
  char output[100];

  int result = 0;
  int forward = 0;

  const char *log_buffer[2];

#ifdef ENABLE_ELOG
  int log_elog;
  log_elog = (toupper(buffer[0]) == buffer[0]);
#endif

  printf("EXE %s\n", buffer);

  switch (buffer[0]) {
    case 'P':
    case 'p':
      forward = 1;
      break;
    case 'L':
    case 'l':
      t_link = 0x00;
      break;
    case 'T':
    case 't':
      t_link = 0x01;
      break;
    case 'I':
    case 'i':
      t_link = 0x02;
      break;
    default:
      result = 11;
  }

  switch (buffer[1]) {
    case '1':
      t_route = 0x09;
      break;
    case '2':
      t_route = 0x0C;
      break;
    default:
      result = 11;
  }

#ifdef ENABLE_ELOG
  // NB: need to use command buffer before Route calls, as they change it
  if (log_elog) {
    ElogBeforeRoute(buffer, user);
  }
#endif

  StoreDefaultParameters(&buffer[3]);

  log_buffer[0] = user;
  log_buffer[1] = buffer;
  WriteLogFile(2, log_buffer, -1);

  if (result == 0) {
    if (link_disabled[(int)buffer[0]][(int)buffer[1] - '1'])
      result = 20; /* channel disabled by command server */
    else if (forward) {
      result = ForwardRoute(buffer);
    } else if (route)
      result = SimpleRoute(sock, fd, &buffer[3]);
    else
      result = SIPRoute(sock, t_link, t_route, &buffer[3]);
  }

  if (err_message[0]) { /* parameter validation failed */
    sprintf(output, ":::ack:::%i:::%s\r\n", result, err_message);
  } else {
    sprintf(output, ":::ack:::%i\r\n", result);
  }
  printf("%i<--%s", sock, output);
  send(sock, output, strlen(output), MSG_NOSIGNAL);

#ifdef ENABLE_ELOG
  if (log_elog) {
    ElogAfterRoute(result, lastsock);
  }
#endif

  return result;
}

void SendCmdDefault(int sock, double d){
  char output[512];

  sprintf(output, ":::cmddef:::%10g\r\n", d);
  printf("%i<--%s", sock, output);
  send(sock, output, strlen(output), MSG_NOSIGNAL);
}

void SendCommandList(int sock)
{
  uint16_t u16;
  int i, j, k;
  char output[4096];

  sprintf(output, ":::rev:::%s\r\n", command_list_serial);
  printf("%i<--%s", sock, output);
  send(sock, output, strlen(output), MSG_NOSIGNAL);

  u16 = N_SCOMMANDS;
  if (send(sock, &u16, sizeof(u16), MSG_NOSIGNAL) < 1)
    return;
  u16 = N_MCOMMANDS;
  if (send(sock, &u16, sizeof(u16), MSG_NOSIGNAL) < 1)
    return;
  if (send(sock, &scommands, sizeof(struct scom) * N_SCOMMANDS, MSG_NOSIGNAL)
      < 1)
  {
    return;
  }
  if (send(sock, &mcommands, sizeof(struct mcom) * N_MCOMMANDS, MSG_NOSIGNAL)
      < 1)
  {
    return;
  }

  for (i = 0; i < N_MCOMMANDS; ++i) {
    for (j = 0; j < mcommands[i].numparams; ++j)
      if (mcommands[i].params[j].nt) {

        u16 = i;
        if (send(sock, &u16, sizeof(u16), MSG_NOSIGNAL) < 1)
          return;

        u16 = j;
        if (send(sock, &u16, sizeof(u16), MSG_NOSIGNAL) < 1)
          return;

        /*
        u16 = i * 256 + j;
        if (send(sock, &u16, sizeof(u16), MSG_NOSIGNAL) < 1)
          return;
        */
        
        /* count */
        for (u16 = 0; mcommands[i].params[j].nt[u16]; ++u16)
          ;
        if (send(sock, &u16, sizeof(u16), MSG_NOSIGNAL) < 1)
          return;

        output[0] = 0;
        for (k = 0; mcommands[i].params[j].nt[k]; ++k) {
          strncat(output, mcommands[i].params[j].nt[k], 79);
          strcat(output, "\n");
        }
        if (send(sock, output, strlen(output), MSG_NOSIGNAL) < 1)
          return;
      } 
  }
  u16 = 0xFFFF;
  if (send(sock, &u16, sizeof(u16), MSG_NOSIGNAL) < 1)
    return;
}

void SendGroupNames(int sock)
{
  int i;
  int num_groups = 0;
  char output[4096];

  output[0] = '\0';
  for (i = 0; i < N_GROUPS && GroupNames[i]; ++i) {
    if (GroupNames[i]) {
      num_groups++;
      strncat(output, GroupNames[i], 127);
      strcat(output, "\n");
    }
  }

  if (send(sock, &num_groups, sizeof(num_groups), MSG_NOSIGNAL) < 1) return;

  send(sock, output, strlen(output), MSG_NOSIGNAL);
}

int MakeSock(int port)
{
  int sock, n;
  struct sockaddr_in addr;

  if ((sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP)) == -1) {
    perror("Unable to create socket");
    exit(15);
  }

  n = 1;
  if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &n, sizeof(n)) != 0) {
    perror("Unable to set socket options");
    exit(15);
  }

  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  addr.sin_addr.s_addr = INADDR_ANY;

  if (bind(sock, (struct sockaddr*)&addr, (socklen_t)sizeof(addr)) == -1) {
    perror("Unable to bind to port");
    exit(15);
  }

  if (listen(sock, 100) == -1) {
    perror("Unable to listen on port");
    exit(15);
  }

  printf("Listening on port %i.\n", port);

  return sock;
}

#define N_CONN 1024
#define CONN_USER_LEN 1024
void Daemonise(int route, int no_fork, int port, char *daemon_pilot[2])
{
  struct {
    int state;
    int report;
    int lurk; /* 0 = off, 1 = on, 2 = request, 3 = cmd_announce */
    int spy;  /* 0 = off, 1 = on, 2 = request, 3 = client_announce */
    char user[CONN_USER_LEN];
  } conn[N_CONN];
  /* state list
   *  0 = client just connected
   *  1 = client user known
   *  2 = client has taken/given/knows conn
   *  3 = client requested group names
   *  4 = client requested command list
   *  5 = client request denied
   *  8 = client unauthorized
   * 0x100 bit set = ping
   */ 

  int owner = 0;

  char buffer[BUF_SIZE];
  char tmp_buf[512];
  char cmd[512];
  int cmd_from = 0;
  int ack = 0;
  double d;
  char dcmd[SIZE_CMDPARNAME];

  int fd, n, i, size, pid, reset_lastsock = 0;
  int report = 0;
  struct sockaddr_in addr;
  int sock, csock, lastsock;
  socklen_t addrlen;
  fd_set fdlist, fdread, fdwrite;

  struct timespec sleep_time;
  sleep_time.tv_sec = 0;
  sleep_time.tv_nsec = 10000000; /* 10ms */

#ifdef USE_AUTHENTICATION
  ReadAuth();	/*read authorized IPs (mainly to check file at this point) */
#endif

  // transfer hosts for forwarding
  if (strlen(daemon_pilot[0]) > 0) {
    pilot_host[0] = daemon_pilot[0];
    pilot_host[1] = daemon_pilot[1];
  }

  printf("HOSTS %s %s\n", pilot_host[0], pilot_host[1]);

  /* for forward management */
  setenv("USER", "blastcmd", 1);

  /* open our output before daemonising just in case it fails. */
  if (route == 1) /* fifo */
    fd = open("/data/etc/SIPSS.FIFO", O_RDWR); 
  else if (route == 2) /* null */
    fd = open("/dev/null", O_WRONLY);
  else
    fd = bc_setserial();

  if (fd < 0) {
    perror("Unable to open output device");
    exit(2);
  }

  /* start the listener. */
  lastsock = sock = MakeSock(port);

  signal(SIGCHLD, SIG_IGN); /* We don't wait for children so don't require
                               the kernel to keep zombies.                  */

  if (!no_fork) {
    /* Fork to background */
    if ((pid = fork()) != 0) {
      if (pid == -1) {
        perror("Unable to fork to background");
        exit(-1);
      }

      printf("PID = %i\n", pid);
      exit(0);
    }

    /* Daemonise */
    if (chdir("/") < 0) perror("chdir failed");
    if (!freopen("/dev/null", "r", stdin)) perror("freopen stdin failed");
    if (!freopen("/dev/null", "w", stdout)) perror("freopen stdout failed");
    if (!freopen("/dev/null", "w", stderr)) perror("freopen stderr failed");
    setsid();
  }

  /* Zero everything */
  memset(conn, 0, sizeof(conn[0]) * N_CONN);

  /* select */
  FD_ZERO(&fdlist);
  FD_SET(sock, &fdlist);

  for (;;) {
    fdwrite = fdread = fdlist;
    FD_CLR(sock, &fdwrite);

    if (reset_lastsock) {
      reset_lastsock = 0;
      for (i = 0; i < sizeof(fd_set) * 8; ++i)
        if (FD_ISSET(i, &fdlist))
          lastsock = i;
    }

    if (report) {
      reset_lastsock = 0;
      for (i = 0; i < sizeof(fd_set) * 8; ++i)
        if (i != sock && FD_ISSET(i, &fdlist))
          conn[i].report = 1;

      report = 0;
    }

    n = pselect(lastsock + 1, &fdread, &fdwrite, NULL, &sleep_time, NULL);

    if (n == -1 && errno == EINTR)
      continue; /* timeout on select */
    else if (n == -1) 
      perror("select");
    else

      /* loop through all sockets, looking for ones that have been returned by
       * select */
      for (n = 0; n <= lastsock; ++n) {
        if (FD_ISSET(n, &fdread)) { /* socket n is waiting for read */
          if (n == sock) { /* read from the listener */
            addrlen = sizeof(addr);
            if ((csock = accept(sock, (struct sockaddr*)&addr, &addrlen)) == -1)
              perror("accept");
            else {
              FD_SET(csock, &fdlist);
              if (csock > lastsock)
                lastsock = csock;
              printf("connect from %s accepted on socket %i\n",
                  inet_ntoa(addr.sin_addr), csock);
#ifdef USE_AUTHENTICATION
              if (IsAuth(&addr.sin_addr))
              {
                printf("Autentication OK from client.\n");
                conn[csock].spy = conn[csock].lurk = conn[csock].state = 0;
              } else {
                printf("Failed authentication from client.\n");
                conn[csock].state = 8;
              }
#else
              conn[csock].spy = conn[csock].lurk = conn[csock].state = 0;
#endif
            }
          } else if (conn[n].state != 8) { /* read from authorised client */
            if ((size = recv(n, buffer, BUF_SIZE, 0)) == -1) {
              perror("recv");
              buffer[0] = 0;
            } else if (size == 0) { /* connection closed */
              printf("connexion dropped on socket %i\n", n);
              shutdown(n, SHUT_RDWR);
              close(n);
              FD_CLR(n, &fdlist);
              FD_CLR(n, &fdwrite);
              reset_lastsock = 1;
              if (owner == n) {
                printf("Conn dropped!\n");
                report = 1;
                owner = 0;
              }
              conn[n].state = 0; /* Note that the connection is closed */
              for (i = 0; i < N_CONN; i++)
                if (conn[i].spy == 1)
                  conn[i].spy = 3;
              continue;
            } else {
              buffer[BUF_SIZE - 1] = '\0';
              for (i = 0; i < BUF_SIZE - 1; ++i)
                if (buffer[i] == '\n' || buffer[i] == '\r') {
                  buffer[i] = '\0';
                  break;
                }
            }

            printf("%i-->%s\n", n, buffer);

            if (conn[n].state == 0) { /* authentication */
              if (strncmp(buffer, "::user::", 8) == 0) {
                conn[n].state = 1;
                strncpy(conn[n].user, buffer + 8, CONN_USER_LEN);
                conn[n].user[CONN_USER_LEN - 1] = 0;
                printf("Authentication on socket %i by %s.\n", n, conn[n].user);
                for (i = 0; i < N_CONN; i++) /* Remember to tell moles about new user */
                  if (conn[i].spy == 1)
                    conn[i].spy = 3;
              } else { /* failed authentication */
                printf("Failed authentication on socket %i.  Closing "
                    "connection.\n", n);
                shutdown(n, SHUT_RDWR);
                close(n);
                FD_CLR(n, &fdlist);
                reset_lastsock = 1;
                if (owner == n) {
                  printf("Conn dropped!\n");
                  report = 1;
                  owner = 0;
                }
              }
            } else if (strncmp(buffer, "::ping::", 8) == 0) {
              conn[n].state |= 0x100;
            } else if (strncmp(buffer, "::lurk::", 8) == 0) {
              conn[n].lurk = 2;
            } else if (strncmp(buffer, "::spy::", 7) == 0) {
              conn[n].spy = 2;
            } else if (strncmp(buffer, "::group::", 9) == 0) {
              conn[n].state = 3;
            } else if (strncmp(buffer, "::list::", 8) == 0) {
              conn[n].state = 4;
            } else if (strncmp(buffer, "::cmddef::", 10) == 0) {
              conn[n].state = 6;
              strncpy(dcmd, buffer+10, SIZE_CMDPARNAME);
            } else if (owner != n) { /* no conn */
              if (strncmp(buffer, "::take::", 8) == 0) {
                printf("Socket %i has taken the conn.\n", n);
                report = 1;
                owner = n;
                conn[n].state = 2;
              } else
                conn[n].state = 5;
            } else if (owner == n) { /* has conn */
              if (strncmp(buffer, "::give::", 8) == 0) {
                printf("Socket %i has given up the conn.\n", n);
                report = 1;
                owner = 0;
                conn[n].state = 2;
              } else if (buffer[0] != ':') {
                ack = ExecuteCommand(n, fd, route, buffer, conn[n].user,
                    lastsock);
                cmd_from = n;
                strncpy(cmd, buffer, 512);
                cmd[511] = 0;
                for (i = 0; i < N_CONN; i++)
                  if (i != n && conn[i].lurk)
                    conn[i].lurk = 3;
              }
            }
          }
        } /* read */

        if (FD_ISSET(n, &fdwrite))    /* connection n available for write */
          if (n != sock) {            /* don't write to the listener */
            buffer[0] = 0;

            /* compose message */
            if (conn[n].state == 1) { /* need to know who has the conn */
              if (owner == 0)
                strcpy(buffer, ":::free:::\r\n");
              else {
                strcpy(buffer, ":::conn:::");
                strcat(buffer, conn[owner].user);
                strcat(buffer, "\r\n");
              }
              conn[n].state = 2;
              conn[n].report = 0;
            } else if (conn[n].state == 3) { /* list */
              SendGroupNames(n);
              conn[n].state = 2;
            } else if (conn[n].state == 4) { /* list */
              SendCommandList(n);
              conn[n].state = 2;
            } else if (conn[n].state == 5) { /* request with no conn */
              strcpy(buffer, ":::noconn:::\r\n");
              conn[n].state = 1;
            } else if (conn[n].state == 6) { /* get command default */
              d = getDefault(dcmd);
              SendCmdDefault(n, d);
              conn[n].state = 2;
            } else if (conn[n].lurk == 2) { /* request for lurking */
              strcpy(buffer, ":::slink:::\r\n");
              conn[n].lurk = 1;
            } else if (conn[n].state & 0x100) { /* ping pong */
              strcpy(buffer, ":::pong:::\r\n");
              conn[n].state &= ~0x100;
            } else if (conn[n].lurk == 3) { /* report command */
              snprintf(buffer, BUF_SIZE, ":::sender:::%s\r\n:::cmd:::%s\r\n:::rep:::%d\r\n", 
                       conn[cmd_from].user, cmd, ack);
              conn[n].lurk = 1;
              buffer[BUF_SIZE - 1] = 0;
            } else if (conn[n].spy == 3) { /* report clients */
              for (i = 0; i < N_CONN; i++) {
                if (conn[i].state) {
                  snprintf(tmp_buf, 100, ":::here:::%s\r\n", conn[i].user);
                  strncat(buffer + strlen(buffer), tmp_buf, BUF_SIZE - strlen(buffer));
                }
              }
              conn[n].spy = 1;
            } else if (conn[n].spy == 2) { /* request for spying */
              strcpy(buffer, ":::mole:::\r\n");
              conn[n].spy = 3;
            } else if (conn[n].state == 8) /* authentication failed */
              strcpy(buffer, ":::nope:::\r\n");

            /* send */
            if (buffer[0]) {
              printf("%i<--%s", n, buffer);
              if ((size = send(n, buffer, strlen(buffer), MSG_NOSIGNAL))
                  == -1) {
                if (errno == EPIPE) {
                  printf("connexion dropped on socket %i\n", n);
                  shutdown(n, SHUT_RDWR);
                  close(n);
                  FD_CLR(n, &fdlist);
                  reset_lastsock = 1;
                  if (owner == n) {
                    printf("Conn dropped!\n");
                    report = 1;
                    owner = 0;
                  }
                } else if (errno != EAGAIN) /* ignore socket buffer overflows */
                  perror("send");
              }
            }

            /* close unauthorised connexions */
            if (conn[n].state == 8) {
              shutdown(n, SHUT_RDWR);
              close(n);
              FD_CLR(n, &fdlist);
              FD_CLR(n, &fdwrite);
              reset_lastsock = 1;
              if (owner == n) {
                printf("Conn dropped!\n");
                report = 1;
                owner = 0;
              }
              continue;
            }

            if (conn[n].state == 2 && conn[n].report) {
              conn[n].report = 0;
              conn[n].state = 1;
            }
          } /* n != sock */
      } /* socket loop */

    nanosleep(&sleep_time, NULL);
  } /* main loop */
}
