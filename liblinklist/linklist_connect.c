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

#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>

#include <errno.h>
#include <netdb.h>
#include <signal.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <inttypes.h>
#include <termios.h>

#include <sys/socket.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <dirent.h>

#include <netinet/in.h>
#include <arpa/inet.h>

#include <pthread.h> // threads
#include <openssl/md5.h>
#include <regex.h>

#include "linklist.h"
#include "linklist_writer.h"
#include "linklist_connect.h"

#define DATAETC "/data/etc"
#define MAX_CONNECT_TRIES 5

#ifndef MIN 
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif

#ifdef __cplusplus
extern "C" {
#endif

extern int (*linklist_info)(const char *, ...);
extern int (*linklist_err)(const char *, ...);
extern int (*linklist_warn)(const char *, ...);
extern int (*linklist_fatal)(const char *, ...);


uint16_t theday = 0;
uint16_t themonth = 0;
uint16_t theyear = 0;
unsigned int clientport = CLIENT_TELEM_PORT;
unsigned int serverport = CLIENT_TELEM_PORT;

/* packet header (12 bytes)
 * ----------------
 * [0-3] = unique sender recver serial
 * [4-7] = frame number
 * [8-9] = packet number
 * [10-11] = total number of packets
 */

void writeTCPHeader(uint8_t * header, uint32_t serial, uint32_t frame_num, uint16_t i_pkt, uint16_t n_packets) {
  // build header
  *((uint32_t *) (header+0)) = serial;
  *((uint32_t *) (header+4)) = frame_num;
  *((uint16_t *) (header+8)) = i_pkt;
  *((uint16_t *) (header+10)) = n_packets;
}

void readTCPHeader(uint8_t * header, uint32_t **ser, uint32_t **frame_num, uint16_t **i_pkt, uint16_t **n_pkt) {
  // extract header
  *ser = (uint32_t *) (header+0);
  *frame_num = (uint32_t *) (header+4);
  *i_pkt = (uint16_t *) (header+8);
  *n_pkt = (uint16_t *) (header+10);
}

static int one (const struct dirent *unused)
{
  if (unused) return 1;
  return 1;
}

// opens a dialog to select file by name from the server
void user_file_select(linklist_tcpconn_t * tc, char *linklistname)
{
  char name[LINKLIST_MAX_FILENAME_SIZE][LINKLIST_SHORT_FILENAME_SIZE] = {{0}};
  int numlink = request_server_archive_list(tc,name);

  if (!numlink) {
    linklistname[0] = 0;
    return;
  }

  int i,j;


  // display all the entries that match the optionally supplied linklist name
  int match = strlen(linklistname);
  int n_match =  numlink;
  char str_match[LINKLIST_MAX_FILENAME_SIZE] = {0};
  strcpy(str_match, linklistname);
  
  regex_t regex;

  if (match) {
    n_match = 0;
    if (regcomp(&regex, str_match, 0)) {
      linklist_err("Regex failed\n");
      return;
    }
    for (i=0;i<numlink;i++) {
      if (regexec(&regex, name[i], 0, NULL, 0) != REG_NOMATCH) {
        strncpy(linklistname, name[i], LINKLIST_MAX_FILENAME_SIZE);

        char temp[LINKLIST_SHORT_FILENAME_SIZE];
        snprintf(temp, LINKLIST_SHORT_FILENAME_SIZE, "%s", name[n_match]);
        snprintf(name[n_match], LINKLIST_SHORT_FILENAME_SIZE, "%s", name[i]);
        snprintf(name[i], LINKLIST_SHORT_FILENAME_SIZE, "%s", temp);

        n_match++;
      }
    }
    regfree(&regex);
    linklist_info("Found %d matches for regex entry \"%s\"\n\n", n_match, str_match);
  }

  // don't have exactly 1 matching entry, so prompt user to select
  if ((n_match != 1) || (strlen(linklistname) == 0)) {
    // deal with display widths
    if (n_match) numlink = n_match;
    int n = (numlink-1)/3+1;
    int width[3] = {0};

    for (i=0;i<n;i++) {
      if ((int) strlen(name[i]) > width[0]) width[0] = strlen(name[i]);
      if ((int) strlen(name[i+n]) > width[1]) width[1] = strlen(name[i+n]);
      if ((int) strlen(name[i+n+n]) > width[2]) width[2] = strlen(name[i+n+n]);
    }
    for (i=0;i<3;i++) width[i] += 3;


    linklist_info("\nSelect archive file:\n\n");

    for (i=0;i<n;i++) {
      if (name[i][0]) printf("%.2d: %s",i,name[i]);
      for (j = strlen(name[i]); j < width[0]; j++) printf(" ");
      if (name[i+n][0]) printf("%.2d: %s",i+n,name[i+n]);
      for (j = strlen(name[i+n]); j < width[1]; j++) printf(" ");
      if (name[i+n+n][0]) printf("%.2d: %s",i+n+n,name[i+n+n]);
      printf("\n");
    }

    while (1) {
      char ta[10];
      printf("\nFile number: ");
      if (fscanf(stdin,"%s",ta) == 0) {
        linklist_err("Unable to parse file number\n");
      }
      int cn = atoi(ta);
      if ((cn >= 0) && (cn < numlink)) {
        strncpy(linklistname, name[cn], LINKLIST_MAX_FILENAME_SIZE);
        break;
      }
      linklist_info("\nInvalid selection\n");
    } 
  }
  
  linklist_info("Archive file \"%s\" selected\n", linklistname);
}

int copy_file(char *old_filename, char *new_filename)
{
  FILE  *ptr_old, *ptr_new;
  int a;

  if (strcmp(old_filename, new_filename) == 0) return 0;

  ptr_old = fopen(old_filename, "rb");
  ptr_new = fopen(new_filename, "wb");

  if (!ptr_old) {
    linklist_err("Can't open %s\n", old_filename);
    return -1;
  }

  if (!ptr_new) {
    linklist_err("Can't open %s\n", new_filename);
    fclose(ptr_old);
    return  -1;
  }

  while (1) {
    a = fgetc(ptr_old); 
    if (!feof(ptr_old)) fputc(a, ptr_new);
    else break;
  }

  fclose(ptr_new);
  fclose(ptr_old);
  return 0;
}


void set_linklist_client_port(unsigned int port) {
  clientport = port;
}
void set_linklist_server_port(unsigned int port) {
  serverport = port;
}

// a macro function that requests format and link files and parses them;
// this function overwrites all previously parsed telemlist format file
// and linklist link files
uint32_t sync_with_server(struct TCPCONN * tc, char * selectname, char * linklistname, unsigned int flags,
                          superframe_t ** sf, linklist_t ** ll)
{
  // initiate server connection if not done already
  while (tc->fd <= 0) {
    tc->fd = connect_tcp(tc);
    sleep(1);
  }

  uint32_t recv_ff_serial, recv_ll_serial;

  char reqffname[LINKLIST_MAX_FILENAME_SIZE] = {0};
  char reqllname[LINKLIST_MAX_FILENAME_SIZE] = {0};
  char reqcsname[LINKLIST_MAX_FILENAME_SIZE] = {0};
  char pathname[LINKLIST_MAX_FILENAME_SIZE] = {0};
  int calspecs = 1;

  // get linklist format file
  while (1) {
    sleep(1);

    snprintf(reqffname, LINKLIST_MAX_FILENAME_SIZE, "%s" SUPERFRAME_FORMAT_EXT, selectname); // suffix for formatfile
    snprintf(reqllname, LINKLIST_MAX_FILENAME_SIZE, "%s" LINKLIST_FORMAT_EXT, selectname); // suffix for linkfile
    snprintf(reqcsname, LINKLIST_MAX_FILENAME_SIZE, "%s" CALSPECS_FORMAT_EXT, selectname); // suffix for calspecs file

    // get the formatfile name
    recv_ff_serial = request_server_file(tc, reqffname, flags);
    if (recv_ff_serial == 0x1badfeed) { // file not found
      linklist_err("Failed to recv superframe format\n");
      user_file_select(tc, selectname);
      continue;
    } else if (recv_ff_serial == 0) { // connection issue
      close_connection(tc);
      tc->fd = connect_tcp(tc);
      continue;
    }

    // get the linkfile name
    recv_ll_serial = request_server_file(tc, reqllname, flags);
    if (recv_ll_serial == 0x1badfeed) { // file not found
      linklist_err("Failed to recv linklist format\n");
      user_file_select(tc, selectname);
      continue;
    } else if (recv_ll_serial == 0) { // connection issue
      close_connection(tc);
      tc->fd = connect_tcp(tc);
      continue;
    }

    // get the calspecs name
    uint32_t temp_ll_serial = request_server_file(tc, reqcsname, flags);
    if (temp_ll_serial == 0x1badfeed) { // file not found
      calspecs = 0; 
    } else if (temp_ll_serial == 0) { // connection issue
      close_connection(tc);
      tc->fd = connect_tcp(tc);
      continue;
    }

    break;
  }

  // parse the superframe format
  snprintf(pathname, LINKLIST_MAX_FILENAME_SIZE, "%s/%s", "/tmp", reqffname);
  if (!(*sf = parse_superframe_format(pathname))) {
    linklist_err("Superframe format parser error.\n");
    return 0;
  }
  // linklist_info("Parsed superframe format \"%s\"\n", pathname);
  unlink(pathname);

  // parse the linklist format
  snprintf(pathname, LINKLIST_MAX_FILENAME_SIZE, "%s/%s", "/tmp", reqllname);
  if (!(*ll = parse_linklist_format(*sf, pathname))) {
    linklist_err("Linklist format parser error.\n");
    return 0;
  }
  // linklist_info("Parsed linklist format \"%s\"\n", pathname);
  unlink(pathname);

  // set the name assigned by the server
  set_server_linklist_name(tc, selectname);
  // linklist_info("Linklist name set to %s\n", selectname);

  // request linklist name resolution if necessary (for symlinks on server)
  request_server_linklist_name(tc, linklistname, LINKLIST_MAX_FILENAME_SIZE, flags & TCPCONN_RESOLVE_NAME); 
  // linklist_info("Linklist name resolves to %s\n", linklistname);

  // parse the calspecs format
  if (calspecs) {
    char fname[LINKLIST_MAX_FILENAME_SIZE];
    snprintf(pathname, LINKLIST_MAX_FILENAME_SIZE, "%s/%s", "/tmp", reqcsname);
    snprintf(fname, LINKLIST_MAX_FILENAME_SIZE, "%s/%s" CALSPECS_FORMAT_EXT, archive_dir, linklistname);

    if (copy_file(pathname, fname) < 0) {
      linklist_err("Cannot parse calspecs format \"%s\"\n", pathname);
    } else {
      // linklist_info("Parsed calspecs format \"%s\"\n", pathname);
    }
    strncpy((*sf)->calspecs, fname, LINKLIST_MAX_FILENAME_SIZE);
    if (strcmp(pathname, fname)) unlink(pathname);
  } else {
    (*sf)->calspecs[0] = '\0';
  }  

  return recv_ll_serial;
}

char *get_real_file_name(char * real_name, char * symlink_name) 
{
  char resolved_name[LINKLIST_MAX_FILENAME_SIZE] = {0};
  char full_name[LINKLIST_MAX_FILENAME_SIZE] = {0};
  snprintf(full_name, LINKLIST_MAX_FILENAME_SIZE, "%s/%s" LINKLIST_EXT ".00", archive_dir, symlink_name);

  if (!realpath(full_name, resolved_name)) {
    real_name[0] = '\0';
    return NULL;
  }
  int i = 0;
  for (i = strlen(resolved_name)-1; i >= 0; i--) {
    if (resolved_name[i] == '/') break;
  }
  strncpy(real_name, resolved_name+i+1, LINKLIST_MAX_FILENAME_SIZE);

  // strip the file extension
  for (i = 0; i < (int) strlen(real_name); i++) {
    if (real_name[i] == '.') break;
  }
  real_name[i] = '\0';

  return real_name;
}

void *connection_handler(void *arg)
{
  //Get the socket descriptor
  int sock = (uintptr_t) arg;

  struct TCPCONN tc = {0};
  tc.fd = sock;

  int read_size;
  char client_message[TCP_PACKET_HEADER_SIZE];
  uint8_t header[TCP_PACKET_HEADER_SIZE] = {0};
  uint32_t *req_frame_num;
  uint32_t *req_serial;
  uint16_t *req_i;
  uint16_t *req_n;
  uint8_t *buffer = NULL;
  unsigned int buffersize = 0;

  int client_on = 1;
  int frame_lag = 1; 

  char linklist_name[LINKLIST_MAX_FILENAME_SIZE] = {0};

  char * archive_filename;
  char archive_symlink_filename[LINKLIST_MAX_FILENAME_SIZE] = {0}; // path to the symlink
  char archive_resolved_filename[LINKLIST_MAX_FILENAME_SIZE] = {0}; // path the file that the symlink resolves to
  char linklist_resolved_name[LINKLIST_MAX_FILENAME_SIZE] = {0}; // name of the file that the symlink resolves to
  linklist_rawfile_t * archive_rawfile = NULL;
  int archive_framenum = 0;
  int archive_lag_framenum = 0;
  int client_flags = 0;

  linklist_info("::SERVER:: Accept CLIENT %d\n",sock);

  while (client_on) {
    // recv header request from client
    read_size = recv(sock, client_message, TCP_PACKET_HEADER_SIZE, 0);

    if (read_size <= 0) {
      linklist_info("::CLIENT %d:: connection dropped\n",sock);
      client_on = 0;
      break;
    }
    readTCPHeader((uint8_t *) client_message, &req_serial, &req_frame_num, &req_i, &req_n);

    // process type of request
    if (*req_serial == SERVER_LL_REQ) { // client requesting superframe/linklist format file to be sent
      // recv name: *req_frame_num is number of characters
      char req_name[LINKLIST_MAX_FILENAME_SIZE] = {0};
      read_size = recv(sock, req_name, MIN(*req_frame_num, LINKLIST_MAX_FILENAME_SIZE), 0);
    
      if (read_size <= 0) {
        linklist_err("::CLIENT %d:: unable to receive linklist name\n",sock);
        client_on = 0;
        break;
      }

      linklist_info("::CLIENT %d:: request for file transfer\n", sock);        

      // strip name of slashes
      int i;
      for (i = strlen(req_name)-1; i >= 0; i--) {
        if (req_name[i] == '/') {
          req_name[i] = 0;
          break;
        }
      }
      i++;

      // build filename in data backup directory
      char filename[LINKLIST_MAX_FILENAME_SIZE] = {0};
      snprintf(filename, LINKLIST_MAX_FILENAME_SIZE, "%s/%s", archive_dir, req_name+i);
      if (send_client_file(&tc, filename, SERVER_ARCHIVE_REQ) < 0) {
        client_on = 0;
        break;
      }
    } else if (*req_serial == SERVER_ARCHIVE_LIST_REQ) { // client requesting list of archived files
      linklist_info("::CLIENT %d:: request for archive file list\n", sock);

      struct dirent **dir;
      int n = scandir(archive_dir, &dir, one, alphasort);
      int i;
      int ifile = 0;
      int nfile = 0;

      if (n < 0) {
        linklist_err("::CLIENT %d:: unable to open archive directory\n", sock);
        client_on = 0;
        break;
      }

      // load archive names
      for (i = 0; i < n; i++) {
        int pos;
        for (pos = 0; pos < (int) strlen(dir[i]->d_name); pos++) {
          if (dir[i]->d_name[pos] == '.') break;
        }
        char tempc[LINKLIST_MAX_FILENAME_SIZE];
        snprintf(tempc, LINKLIST_MAX_FILENAME_SIZE, LINKLIST_EXT ".00");

        // look for linklist binary files
        if (strcmp(dir[i]->d_name+pos, tempc) == 0) {
          dir[i]->d_name[pos] = 0; // clear the suffix

          // check if corresponding superframe format and linklist format files exist
          snprintf(tempc, LINKLIST_MAX_FILENAME_SIZE, "%s/%s" LINKLIST_FORMAT_EXT, archive_dir, dir[i]->d_name);
          if (access(tempc, F_OK) != 0) {
            dir[i]->d_name[0] = 0;
            continue;
          }
          snprintf(tempc, LINKLIST_MAX_FILENAME_SIZE, "%s/%s" SUPERFRAME_FORMAT_EXT, archive_dir, dir[i]->d_name);
          if (access(tempc,F_OK) != 0) {
            dir[i]->d_name[0] = 0;
            continue;
          }

          // strip the forward slashes          
          int j;
          for (j=strlen(dir[i]->d_name)-1;j>=0;j--) {
            if (dir[i]->d_name[j] == '/') break;
          }
          strcpy(tempc, dir[i]->d_name+j+1); // copy the abbr. name
          strcpy(dir[i]->d_name, tempc); // copy it back          
 
          nfile++;
        } else {
          dir[i]->d_name[0] = 0;
        }
      }

      ifile = 0;
      for (i = 0; i < n; i++) {
        if (strlen(dir[i]->d_name) <= 0) continue;

        // respond with header for list transfer
        // format: SERVER_ARCHIVE_LIST_REQ, num chars, file index, total # of files
        writeTCPHeader(header, SERVER_ARCHIVE_LIST_REQ, strlen(dir[i]->d_name)+1, ifile, nfile);
        if (send(sock, header, TCP_PACKET_HEADER_SIZE, MSG_NOSIGNAL) <= 0) {
          linklist_err("::CLIENT %d:: unable to send archive name %s\n", sock, dir[i]->d_name);
          client_on = 0;
        }
        // send name
        if (send(sock, dir[i]->d_name, strlen(dir[i]->d_name)+1, MSG_NOSIGNAL) <= 0) {
          linklist_err("::CLIENT %d:: unable to send archive name %s\n", sock, dir[i]->d_name);
          client_on = 0;
        }
        ifile++;
      }
      if (!ifile && !nfile) {
        writeTCPHeader(header, SERVER_ARCHIVE_LIST_REQ, 0, 0, 0);
        if (send(sock, header, TCP_PACKET_HEADER_SIZE, MSG_NOSIGNAL) <= 0) {
          linklist_err("::CLIENT %d:: unable to send archive names\n", sock);
          client_on = 0;
        }
      
      }

      linklist_info("::CLIENT %d:: sent %d file names\n", sock, nfile); 

    } else if (*req_serial == SERVER_SET_LL_NAME_REQ) { // client wants to set the linklist name
      // recv name: *req_frame_num is number of characters
      char req_name[LINKLIST_MAX_FILENAME_SIZE] = {0};
      read_size = recv(sock, req_name, MIN(*req_frame_num, LINKLIST_MAX_FILENAME_SIZE), 0);

      if (read_size <= 0) {
        linklist_err("::CLIENT %d:: unable to receive and set linklist name\n", sock);
        client_on = 0;
        break;
      }

      // strip name of slashes
      int j;
      for (j = strlen(req_name)-1; j >= 0; j--) {
        if (req_name[j] == '/') { 
          req_name[j] = 0;
          break;
        }
      }
      j++;

      // this name should include the linklist binary file extension (LINKLIST_EXT)
      strncpy(linklist_name, req_name+j, LINKLIST_MAX_FILENAME_SIZE);
      linklist_info("::CLIENT %d:: linklist name set to \"%s\"\n", sock, linklist_name);
    } else if (*req_serial == SERVER_LL_NAME_REQ) { // client wants the linklist name
      char send_name[LINKLIST_MAX_FILENAME_SIZE] = {0};
      if (*req_i & TCPCONN_RESOLVE_NAME) { // resolve any symlinks in linklist name if necessary
        if (!get_real_file_name(send_name, linklist_name)) {
          linklist_err("::CLIENT %d:: unable to resolve linklist name \"%s\"\n", sock, linklist_name);
          strncpy(send_name, linklist_name, LINKLIST_MAX_FILENAME_SIZE); // default to unresolved name
        }
        client_flags |= TCPCONN_RESOLVE_NAME;
      } else { // just get the linklist name
        strncpy(send_name, linklist_name, LINKLIST_MAX_FILENAME_SIZE);
      }
      // shorten the name for sending to client if necessary
      if (((*req_frame_num)-1) <= strlen(send_name)) send_name[(*req_frame_num)-1] = 0;
      writeTCPHeader(header, SERVER_LL_NAME_REQ, strlen(send_name)+1, 0, 0);
      if (send(sock, header, TCP_PACKET_HEADER_SIZE, MSG_NOSIGNAL) <= 0) {
        linklist_err("::CLIENT %d:: unable to send response to linklist name\n", sock);
        client_on = 0;
        break;
      }
      if (send(sock, send_name, strlen(send_name)+1, MSG_NOSIGNAL) <= 0) {
        linklist_err("::CLIENT %d:: unable to send linklist name %s\n", sock, send_name);
        client_on = 0;
        break;
      }
      linklist_info("::CLIENT %d:: linklist \"%s\" name sent\n", sock, send_name);
    } else if (*req_serial == SERVER_ARCHIVE_REQ) { // client requesting archived data
      if (!archive_rawfile) { // assume archive file has not yet been loaded
        // check if linklist_name has been loaded from format file and linklist file request
        if (strlen(linklist_name) == 0) {
          linklist_err("::CLIENT %d:: no archive file requested\n", sock);
          client_on = 0;
          break;
        }

        // write data file base name for archive file
        char real_name[LINKLIST_MAX_FILENAME_SIZE];
        snprintf(archive_symlink_filename, LINKLIST_MAX_FILENAME_SIZE, "%s/%s", archive_dir, linklist_name);
        snprintf(archive_resolved_filename, LINKLIST_MAX_FILENAME_SIZE, "%s/%s", archive_dir, get_real_file_name(linklist_resolved_name, linklist_name));

        archive_filename = (client_flags & TCPCONN_RESOLVE_NAME) ? archive_resolved_filename : archive_symlink_filename;
        strncpy(linklist_resolved_name, real_name, LINKLIST_MAX_FILENAME_SIZE);

        if ((archive_rawfile = open_linklist_rawfile(archive_filename, NULL)) == NULL) {
          linklist_err("::CLIENT %d:: unable to open archive file \"%s\"\n", sock, archive_filename);
          client_on = 0;
          break;
        }
        linklist_info("::CLIENT %d:: opening rawfile \"%s\"\n", sock, archive_filename);
      }

      // update the archived file framenumber
      char test_symlink_name[LINKLIST_MAX_FILENAME_SIZE] = {0};
      get_real_file_name(test_symlink_name, linklist_name);
      seekend_linklist_rawfile(archive_rawfile);
      archive_framenum = tell_linklist_rawfile(archive_rawfile);
      archive_lag_framenum = archive_framenum-frame_lag;

      // handle the type of data block request 
      if (*req_i & TCPCONN_CLIENT_INIT) { // requesting a frame number initialization
        // initialize variables for live data block transfer
        linklist_info("::CLIENT %d:: request for initialization\n",sock);

        // respond with header for live data
        // format: serial, initialization framenm, day, year*12+month
        writeTCPHeader(header, SERVER_ARCHIVE_REQ, archive_framenum-frame_lag, theday, theyear*12+themonth);
        if (send(sock, header, TCP_PACKET_HEADER_SIZE, MSG_NOSIGNAL) <= 0) {
          linklist_err("::CLIENT %d:: unable to send initialization message\n",sock);
          client_on = 0;
        }
        memset(header, 0, TCP_PACKET_HEADER_SIZE);

        linklist_info("::CLIENT %d:: initialized with serial 0x%x, nof %d, size %d\n", sock, SERVER_ARCHIVE_REQ, archive_framenum, archive_rawfile->framesize);
      } else if (strcmp(linklist_resolved_name, test_symlink_name)) { // the symlink has changed
        // respond with header for change in datafile
        linklist_info("::CLIENT %d:: data file symlink change\n", sock);
        writeTCPHeader(header, SERVER_ARCHIVE_REQ, *req_frame_num, TCPCONN_FILE_RESET, 0);
        if (send(sock, header, TCP_PACKET_HEADER_SIZE, MSG_MORE | MSG_NOSIGNAL) <= 0) {
          linklist_err("::CLIENT %d:: unable to send header\n", sock);
          client_on = 0;
          break;
        }
        close_and_free_linklist_rawfile(archive_rawfile);
        archive_rawfile = NULL;
      } else if ((archive_lag_framenum < (int) (*req_frame_num) || (archive_lag_framenum <= 0))) { // no data to read
        // respond with header for no data
        writeTCPHeader(header, SERVER_ARCHIVE_REQ, *req_frame_num, TCPCONN_NO_DATA, 0);
        if (send(sock, header, TCP_PACKET_HEADER_SIZE, MSG_MORE | MSG_NOSIGNAL) <= 0) {
          linklist_err("::CLIENT %d:: unable to send header\n", sock);
          client_on = 0;
          break;
        }
      } else {
        // respond with header for live data
        writeTCPHeader(header, SERVER_ARCHIVE_REQ, *req_frame_num, 0, 0);
        if (send(sock, header, TCP_PACKET_HEADER_SIZE, MSG_MORE | MSG_NOSIGNAL) <= 0) {
          linklist_err("::CLIENT %d:: unable to send header\n", sock);
          client_on = 0;
          break;
        }

        // buffer allocation 
        if (buffersize < archive_rawfile->framesize) {
          if (!buffersize) { // first allocation of the buffer
            buffer = calloc(1, archive_rawfile->framesize);
          } else { // reallocation of the buffer
            buffer = realloc(buffer, archive_rawfile->framesize);
          }
          // handle buffer allocation errors
          if (!buffer) {
            linklist_err("::CLIENT %d:: cannot allocate buffer\n", sock);
            client_on = 0; 
            break;
          }
          buffersize = archive_rawfile->framesize;
        }

        // read the file
        if (seek_linklist_rawfile(archive_rawfile, *req_frame_num) != 0) {
          linklist_err("::CLIENT %d:: fseek failed\n", sock);
          client_on = 0;
          break;
        }
        read_linklist_rawfile(archive_rawfile, buffer);

        // send the data 
        if (send(sock, buffer, archive_rawfile->framesize, MSG_NOSIGNAL) <= 0) {
          linklist_err("::CLIENT %d:: unable to send data\n",sock);
          break;
        }
        memset(buffer, 0, buffersize);
      }
    }
  }

  linklist_info("::SERVER:: closed connection to CLIENT %d\n",sock);

  // cleanup
  if (archive_rawfile) close_and_free_linklist_rawfile(archive_rawfile);
  archive_rawfile = NULL;
  if (buffer) free(buffer);
  buffer = NULL;
  close(sock);

  return 0;
}

void linklist_server(void * arg)
{
  (void) arg; // not used for now

  int client_sock, c;
  struct sockaddr_in server , client;
  int theport = serverport;

  //Create socket
  int socket_desc = socket(AF_INET , SOCK_STREAM , 0);
  if (socket_desc == -1) {
    perror("socket could not create server socket");
    exit(4);
  }

  //Prepare the sockaddr_in structure
  server.sin_family = AF_INET;
  server.sin_addr.s_addr = INADDR_ANY;
  server.sin_port = htons(theport);

  int tru = 1;
  setsockopt(socket_desc, SOL_SOCKET, SO_REUSEADDR, &tru, sizeof(int));

  //Bind
  if (bind(socket_desc, (struct sockaddr *) &server , sizeof(server)) < 0) {
    //print the error message
    perror("bind failed. Error");
    return;
  }

  //Listen
  listen(socket_desc , 3);


  //Accept and incoming connection
  linklist_info("::SERVER:: Waiting for incoming connections on port %d...\n", theport);
  c = sizeof(struct sockaddr_in);
  pthread_t thread_id;

  while ((client_sock = accept(socket_desc, (struct sockaddr *)&client, (socklen_t*)&c))) {
    if (client_sock < 0) {
      linklist_err("Accept client failed\n");
    }
  
    uintptr_t the_sock = client_sock;
    if (pthread_create(&thread_id, NULL, connection_handler, (void*) the_sock) < 0) {
      perror("Could not create client thread\n");
    }
  }
}

// blocks until establishes a connection with the host at tc->ip
// return the file descriptor, which is also at tc->fd
int connect_tcp(struct TCPCONN * tc)
{
  // start tcp client 
  struct sockaddr_in server_info;
  struct hostent *he;
  int socket_fd = 0;
  int numtries = 0;

  int connected = 0;
  while (!connected) {
    if ((he = gethostbyname(tc->ip))==NULL) {
      linklist_err("Cannot get the host name \"%s\"\n",tc->ip);
      numtries = MAX_CONNECT_TRIES;
      if (tc->flag & TCPCONN_NOLOOP) { // non blocking mode
        return socket_fd;
      }
      goto RETRY;
    }

    if (!(tc->flag & TCPCONN_NOLOOP)) linklist_info("Connecting to %s:%d...\n", tc->ip, clientport);
    connected = 1; 
    if ((socket_fd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
      fprintf(stderr, "Socket Failure!!\n");
      if (tc->flag & TCPCONN_NOLOOP) { // non blocking mode
        return 0;
      }
      goto RETRY;
    }

    int tru = 1;
    setsockopt(socket_fd,SOL_SOCKET,SO_REUSEADDR,&tru,sizeof(int));

    memset(&server_info, 0, sizeof(server_info));
    server_info.sin_family = AF_INET;
    server_info.sin_port = htons(clientport);
    server_info.sin_addr = *((struct in_addr *)he->h_addr);

    // Set non-blocking 
    /*
    long int arg = 0;
    if((arg = fcntl(socket_fd, F_GETFL, NULL)) < 0) { 
      fprintf(stderr, "Error fcntl(..., F_GETFL) (%s)\n", strerror(errno)); 
      exit(0); 
    } 
    arg |= O_NONBLOCK; 
    if( fcntl(socket_fd, F_SETFL, arg) < 0) { 
      fprintf(stderr, "Error fcntl(..., F_SETFL) (%s)\n", strerror(errno)); 
      exit(0); 
    } 
    */

    // connect to the socket
    if (connect(socket_fd, (struct sockaddr *)&server_info, sizeof(struct sockaddr))<0) {
      connected = 0;
      close(socket_fd);
      socket_fd = 0;
      if (tc->flag & TCPCONN_NOLOOP) { // non blocking mode
        return socket_fd;
      }
      linklist_info("Connection refused...trying again\n");
      goto RETRY;
    }

    /*
    // Set to blocking mode again... 
    if( (arg = fcntl(socket_fd, F_GETFL, NULL)) < 0) { 
      fprintf(stderr, "Error fcntl(..., F_GETFL) (%s)\n", strerror(errno)); 
      exit(0); 
    } 
    arg &= (~O_NONBLOCK); 
    if( fcntl(socket_fd, F_SETFL, arg) < 0) { 
      fprintf(stderr, "Error fcntl(..., F_SETFL) (%s)\n", strerror(errno)); 
      exit(0); 
    } 
    */

    // break since all was well
    break;

    RETRY : 
    sleep(2);
    numtries++;

    // try a new host if not in multi mole mode
    if ((numtries >= MAX_CONNECT_TRIES) && !(tc->flag & TCPCONN_LOOP)) {
      linklist_info("\nTry a different host: ");
      if (fscanf(stdin,"%s",tc->ip) == 0) {
        linklist_err("Unable to parse host\n");
      }
      if (tc->ip[strlen(tc->ip)-1] == '\n') tc->ip[strlen(tc->ip)-1] = 0;
      numtries = 0;
    }
  }

  tc->fd = socket_fd; // set the file desc

  return socket_fd;
}

// closes the tcp connection and sets the file descriptor to 0
int close_connection(struct TCPCONN * tc)
{
  if (tc->fd != 0) close(tc->fd);
  tc->fd = 0;
  return tc->fd;
}

void send_client_error(struct TCPCONN * tc)
{
  uint8_t header[TCP_PACKET_HEADER_SIZE] = {0};

  writeTCPHeader(header,0x1badfeed, 0, 0, 1);
  if (send(tc->fd, header, TCP_PACKET_HEADER_SIZE, MSG_NOSIGNAL) <= 0) {
    linklist_err("::CLIENT %d:: could not send client error\n", tc->fd);
    return;
  }
}

// send a file to the client
int send_client_file(struct TCPCONN * tc, char * filename, uint32_t serial)
{
  uint8_t header[TCP_PACKET_HEADER_SIZE] = {0};
  uint8_t buffer[LINKLIST_MAX_FILENAME_SIZE] = {0};  

  // open the requested file
  FILE * req_file = fopen(filename, "r");
  if (req_file == 0) {
    linklist_err("::CLIENT %d:: could not open file \"%s\"\n",tc->fd,filename);
    send_client_error(tc);
    return 0;
  }

  // get file size
  fseek(req_file,0,SEEK_END);
  unsigned int req_filesize = ftell(req_file);
  fseek(req_file, 0, SEEK_SET);

  // respond with header for file transfer
  // format: SERVER_LL_REQ, total file size [bytes], 4 byte linklist serial
  writeTCPHeader(header, SERVER_LL_REQ, req_filesize, (serial & 0xffff), (serial & 0xffff0000)>>16);
  if (send(tc->fd, header, TCP_PACKET_HEADER_SIZE, MSG_NOSIGNAL) <= 0) {
    linklist_err("::CLIENT %d:: unable to send file transfer initialization message\n", tc->fd);
    fclose(req_file);
    return -1;
  }
  memset(header, 0, TCP_PACKET_HEADER_SIZE);

  // send raw linklist file
  unsigned int bytes_transferred = 0;
  unsigned int loc = 0, col = 0;
  while (bytes_transferred < req_filesize) {
    loc = MIN(LINKLIST_MAX_FILENAME_SIZE, req_filesize-bytes_transferred);
    col = fread(buffer, 1, loc, req_file);
    if (loc != col) {
      linklist_err("::CLIENT %d:: file read error\n", tc->fd);
      return 0;
    } else {
      if (send(tc->fd, buffer, col, MSG_NOSIGNAL) <= 0) {
        linklist_err("::CLIENT %d:: file transfer interrupted\n", tc->fd);
        fclose(req_file);
        return -1;
      }
      bytes_transferred += col;
      memset(buffer, 0, col);
    }

  }

  fclose(req_file);

  linklist_info("::CLIENT %d:: file transfer complete (%d bytes)\n", tc->fd, req_filesize);

  return 1;
}


// requests a file from the server
// file is saved at DATAETC/filename
uint32_t request_server_file(struct TCPCONN * tc, char * filename, unsigned int flags)
{
  int sock = tc->fd;
  uint8_t buffer[LINKLIST_MAX_FILENAME_SIZE] = {0};
  uint8_t msg[TCP_PACKET_HEADER_SIZE] = {0};

  uint32_t *recv_ser, *recv_filesize;
  uint16_t *recv_i, *recv_n;
  uint32_t ll_serial;

  char savedname[LINKLIST_MAX_FILENAME_SIZE] = {0};
  snprintf(savedname, LINKLIST_MAX_FILENAME_SIZE, "%s/%s", "/tmp", filename);

  // linklist_info("Requesting file \"%s\" from server...\n",filename);

  // send request for file by name
  writeTCPHeader(buffer, SERVER_LL_REQ, strlen(filename), flags, 0);
  if (send(sock, buffer, TCP_PACKET_HEADER_SIZE, MSG_NOSIGNAL) <= 0) {
    linklist_err("request_server_file: failed to send file request message\n");
    return 0;
  }
  if (send(sock, filename, strlen(filename), MSG_NOSIGNAL) <= 0) {
    linklist_err("request_server_file: failed to send filename\n");
    return 0;
  }

  // get file info before receiving
  if (recv(sock, msg, TCP_PACKET_HEADER_SIZE, MSG_WAITALL) <= 0) {
    linklist_err("request_server_file: failed to recv file stats\n");
    return 0;
  }
  readTCPHeader(msg, &recv_ser, &recv_filesize, &recv_i, &recv_n);
  ll_serial = ((*recv_n)<<16) + (*recv_i);

  if (*recv_ser != SERVER_LL_REQ) { // handle error messages
    linklist_err("request_server_file: file \"%s\" not on server\n",filename);
    return *recv_ser;
  }

  // receive the file and write it to disk
  FILE * f = fopen(savedname,"wb");

  unsigned int bytes_recvd = 0;
  unsigned int loc = 0;
  while (bytes_recvd < *recv_filesize) {
    loc = MIN(LINKLIST_MAX_FILENAME_SIZE,(*recv_filesize)-bytes_recvd);
    if (recv(sock, buffer, loc, MSG_WAITALL) <= 0) {
      linklist_err("request_server_file: failed to recv file block\n");
      return 0;
    }
    fwrite(buffer,1,loc,f);
    bytes_recvd += loc;
    memset(buffer,0,loc);
  }
  fflush(f);
  fclose(f);

  //linklist_info("Received file \"%s\" from server (size %d)\n",filename,*recv_filesize);

  return ll_serial;
}

// initialize server connection with time and framenum info based on a linklist serial
int64_t initialize_client_connection(struct TCPCONN * tc, uint32_t serial)
{
  // initialize connection if not done so already
  while (tc->fd <= 0) {
    tc->fd = connect_tcp(tc);
    sleep(1);
  }

  uint32_t * recv_frame_num = 0;
  uint16_t * recv_i = 0, * recv_n = 0;
  uint32_t * recv_ser;
  uint8_t request_msg[TCP_PACKET_HEADER_SIZE] = {0};

  // initialization with serial
  writeTCPHeader(request_msg, serial, 0, TCPCONN_CLIENT_INIT, 0);
  if (send(tc->fd, request_msg, TCP_PACKET_HEADER_SIZE, MSG_NOSIGNAL) <= 0) {
    linklist_err("Failed to send initialization message\n");
    close_connection(tc);
    tc->fd = connect_tcp(tc);
    return -1;
  }

  if (recv(tc->fd, request_msg, TCP_PACKET_HEADER_SIZE, MSG_WAITALL) <= 0) {
    linklist_err("Failed to receive initalization message\n");
    close_connection(tc);
    tc->fd = connect_tcp(tc);
    return -1;
  }
  readTCPHeader(request_msg, &recv_ser, &recv_frame_num, &recv_i, &recv_n);
  
  tc->theday = *recv_i;
  tc->themonth = ((*recv_n-1)%12)+1;
  tc->theyear = (*recv_n-1)/12;
  
  tc->serial = *recv_ser;

  return *recv_frame_num;
}

void request_server_linklist_name(struct TCPCONN * tc, char * linklistname, unsigned int len, unsigned int flags)
{
  // initiate server connection if not done already
  while (tc->fd <= 0) {
    tc->fd = connect_tcp(tc);
    sleep(1);
  }

  uint8_t request_msg[TCP_PACKET_HEADER_SIZE] = {0};
  uint32_t *recv_serial = NULL;
  uint32_t *recv_frame_num = NULL;
  uint16_t *recv_i = NULL;
  uint16_t *recv_n = NULL;

  // request file list from server
  writeTCPHeader(request_msg, SERVER_LL_NAME_REQ, len-1, flags, 0);
  if (send(tc->fd, request_msg, TCP_PACKET_HEADER_SIZE, MSG_NOSIGNAL) <= 0) {
    linklist_err("Failed to send link name request\n");
    close_connection(tc);
    tc->fd = connect_tcp(tc);
  }

  // receive the name
  if (recv(tc->fd, request_msg, TCP_PACKET_HEADER_SIZE, MSG_WAITALL) <= 0) {
    linklist_err("Failed to receive archive name header\n");
    close_connection(tc);
    tc->fd = connect_tcp(tc);
    return;
  }
  readTCPHeader(request_msg, &recv_serial, &recv_frame_num ,&recv_i,&recv_n);
  if (*recv_serial == SERVER_LL_NAME_REQ) {
    // recv name: *recv_frame_num is number of characters
    memset(linklistname, 0, len);
    if (*recv_frame_num > len)  {
      linklist_err("Could not receive server linklist name of size %d for %d size buffer\n", *recv_frame_num, len);
      return;
    }
    int read_size = recv(tc->fd, linklistname, *recv_frame_num, 0);

    if (read_size <= 0) {
      linklist_err("Could not receive server linklist name\n");
      return;
    }
  } else {
    linklist_err("Improper response to linklist name request (0x%x)\n", *recv_serial);
  }
}

// manually sets the linklist name used by the server
void set_server_linklist_name(struct TCPCONN * tc, char *linklistname)
{
  // initiate server connection if not done already
  while (tc->fd <= 0) {
    tc->fd = connect_tcp(tc);
    sleep(1);
  }

  uint8_t request_msg[TCP_PACKET_HEADER_SIZE] = {0};

  // request file list from server
  writeTCPHeader(request_msg,SERVER_SET_LL_NAME_REQ,strlen(linklistname)+1,0,0);
  if (send(tc->fd, request_msg, TCP_PACKET_HEADER_SIZE, MSG_NOSIGNAL) <= 0) {
    linklist_err("Failed to send link name select request\n");
    close_connection(tc);
    tc->fd = connect_tcp(tc);
  }
  if (send(tc->fd, linklistname, strlen(linklistname)+1, MSG_NOSIGNAL) <= 0)
  {
    linklist_err("Failed to send link name\n");
    close_connection(tc);
    tc->fd = connect_tcp(tc);
  }

}

// fills the name array with archived linklist names from the server
int request_server_archive_list(struct TCPCONN * tc, char name[][LINKLIST_SHORT_FILENAME_SIZE])
{
  // initiate server connection if not done already
  while (1) {
    tc->fd = connect_tcp(tc);

    uint8_t request_msg[TCP_PACKET_HEADER_SIZE] = {0};

    // request file list from server
    writeTCPHeader(request_msg,SERVER_ARCHIVE_LIST_REQ,0,0,0);
    if (send(tc->fd, request_msg, TCP_PACKET_HEADER_SIZE, MSG_NOSIGNAL) <= 0) {
      linklist_err("Failed to send archive select request\n");
      close_connection(tc);
      if (tc->flag & TCPCONN_NOLOOP) return -1;
      continue;
    }

    uint16_t *recv_i, *recv_n;
    uint32_t *recv_ser, *recv_fn;

    while (1) {
      // receive name header
      if (recv(tc->fd,request_msg,TCP_PACKET_HEADER_SIZE,MSG_WAITALL) <= 0) {
        linklist_err("Failed to receive archive name header\n");
				if (tc->flag & TCPCONN_NOLOOP) return -1;
        break;
      }
      readTCPHeader(request_msg,&recv_ser,&recv_fn,&recv_i,&recv_n);

      // no files on server
      if (!(*recv_i) && !(*recv_n)) {
        linklist_err("No files avaiable on server %s\n", tc->ip);
				if (tc->flag & TCPCONN_NOLOOP) return -1;
        break;
      }

      if (*recv_fn > LINKLIST_SHORT_FILENAME_SIZE) {
        linklist_err("Cannot receive archive name of size %d for buffer size %d\n", *recv_fn, LINKLIST_SHORT_FILENAME_SIZE);
        return -1;
      }

      // receive name
      if (recv(tc->fd,name[*recv_i],*recv_fn,MSG_WAITALL) <= 0) {
        linklist_err("Failed to receive archive name\n");
        close_connection(tc);
	if (tc->flag & TCPCONN_NOLOOP) return -1;
        break; 
      }

      // this is the end of the function
      if (((*recv_i)+1) >= *recv_n) {
        return *recv_n;
      }
    }

    // only get here if there was a conneciton problem
    close_connection(tc);
    linklist_err("Unable to connect to server\n");
    sleep(1);
  }

  // should never get here
  return 0;

}

// fills the name array with linklist names from the server
int request_server_list(struct TCPCONN * tc, char name[][LINKLIST_SHORT_FILENAME_SIZE]) {
  // initiate server connection if not done already
  while (tc->fd <= 0)
  {
    tc->fd = connect_tcp(tc);
    sleep(1);
  }

  uint8_t request_msg[TCP_PACKET_HEADER_SIZE] = {0};

  // request file list from server
  writeTCPHeader(request_msg,SERVER_LL_LIST_REQ,0,0,0);
  if (send(tc->fd, request_msg, TCP_PACKET_HEADER_SIZE, MSG_NOSIGNAL) <= 0) {
    linklist_err("Failed to send link select request\n");
    close_connection(tc);
    tc->fd = connect_tcp(tc);
  }

  uint16_t *recv_i, *recv_n;
  uint32_t *recv_ser, *recv_fn;


  while (1)
  {
    // receive name header
    if (recv(tc->fd,request_msg,TCP_PACKET_HEADER_SIZE,MSG_WAITALL) <= 0) {
      linklist_err("Failed to receive linklist name header\n");
      close_connection(tc);
      return 0;
    }
    readTCPHeader(request_msg,&recv_ser,&recv_fn,&recv_i,&recv_n);

    if (*recv_fn > LINKLIST_SHORT_FILENAME_SIZE) {
      linklist_err("Cannot receive archive name of size %d for buffer size %d\n", *recv_fn, LINKLIST_SHORT_FILENAME_SIZE);
      return 0;
    }

    // receive name
    if (recv(tc->fd,name[*recv_i],*recv_fn,0) <= 0) {
      linklist_err("Failed to receive linklist name\n");
      close_connection(tc);
      return 0;
    }  

    if (*recv_n  == ((*recv_i)+1)) break;

  }
  return *recv_n;
}


// requests data from the server
// data is requested based on serial number and framenumber
// returns the framenumber to be retrieved 
int request_data(struct TCPCONN *tc, unsigned int fn, uint16_t * flags) {
  uint8_t request_msg[TCP_PACKET_HEADER_SIZE] = {0};  
  int rsize = 0;

  uint32_t * recv_serial = NULL;
  uint32_t * recv_framenum = NULL;
  uint16_t * recv_i = NULL;
  uint16_t * recv_n = NULL;

  // request the next frame
  writeTCPHeader(request_msg, tc->serial, fn, 0, 0);
  if (send(tc->fd, request_msg, TCP_PACKET_HEADER_SIZE, MSG_NOSIGNAL) <= 0) {
    linklist_err("Server connection lost on send.\n");
		close_connection(tc);
		tc->fd = connect_tcp(tc);
    return -1; // connection error
  }

  // receive header for the next frame
  if ((rsize = recv(tc->fd, request_msg, TCP_PACKET_HEADER_SIZE, MSG_WAITALL)) <= 0) {
    linklist_err("Server connection lost on recv.\n");
		close_connection(tc);
		tc->fd = connect_tcp(tc);
    return -1; // connection error
  }
  readTCPHeader(request_msg, &recv_serial, &recv_framenum, &recv_i, &recv_n); 
  
  // set the flags 
  *flags = *recv_i;

  return *recv_framenum;
}

// retrieves data from the server and writes it to the specified buffer of size bufsize
// data is requested based on serial number and framenumber
// returns the number of retrieved bytes
int retrieve_data(struct TCPCONN * tc, uint8_t * buffer, unsigned int bufsize)
{
  int rsize = 0;

  // receive the next frame
  if ((rsize = recv(tc->fd, buffer, bufsize, MSG_WAITALL)) <= 0) {
    linklist_err("Server connection lost on recv.\n");
    close_connection(tc);
    tc->fd = connect_tcp(tc);
    return -1; // connection error
  }

  return rsize;
}

#ifdef __cplusplus
}
#endif
