/**
 * @file csbf_dgps.c
 *
 * @date Aug 11, 2012
 * @author seth
 *
 * @brief This file is part of FCP, created for the EBEX project
 *
 * This software is copyright (C) 2011 Columbia University
 *
 * FCP is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * FCP is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with FCP; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

// In this file, we parse packets from an auxiliary GPS unit bolted to the SIP.
// Normally, mcp asks the SIP for GPS lat/lon/alt/time via a non-NMEA interface
// and the SIP reports it to us. If we ask CSBF for a GPS compass, they fly a
// boom with 2 GPS antennas on it, connected to the same receiver, and we can
// get a direct serial output from that unit, which reports various NMEA-
// formatted GPS data, including the GPS heading. -ECM


#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>

#include "blast.h"
#include "comms_serial.h"
#include "gps.h"
#include "mcp.h"
#include "pointing_struct.h"
#include "socket_utils.h"
#include "tx.h"

#include "csbf_dgps.h"

#define NMEA_CHATTER 0

// comm port for the CSBF GPS
#define CSBFGPSCOM "/dev/ttyCSBFGPS"
// Parameters for receiving NMEA sentences via UDP
#define GPS_IP_ADDR "192.168.0.115"
#define GPS_PORT "4952"
// Max expected length of any concatenation of UDP GPS sentences from a GPS
// module. The value depends on how many and what types of sentences we
// configure the module to send at once. Chars beyond this length are dropped.
#define MAXLEN_NMEA_0183_MESSAGE 300

void nameThread(const char*);
struct DGPSAttStruct CSBFGPSAz = {.az = 0.0, .att_ok = 0};
struct GPSInfoStruct CSBFGPSData = {.longitude = 0.0};

// Access to GPS data members protected by mutex: could be updated by either/
// both serial and UDP threads
pthread_mutex_t GGAlock;
pthread_mutex_t HDTlock;
pthread_mutex_t ZDAlock;

// TODO(laura): We don't actually do anything with the time read out from the CSBF GPS,
// should we write it to the frame?
time_t csbf_gps_time;

static void processGGA(const char* m_data, const char* fmt_str);
static void processGPGGA(const char *m_data);
static void processGNGGA(const char *m_data);
static void processHDT(const char *m_data, const char* fmt_str);
static void processGPHDT(const char *m_data);
static void processGNHDT(const char *m_data);
static void processZDA(const char* m_data, const char* fmt_str);
static void processGPZDA(const char *m_data);
static void processGNZDA(const char *m_data);

nmea_handler_t handlers[] = {
    { processGNGGA, "$GNGGA," }, // fix info
    { processGPGGA, "$GPGGA," }, // fix info, GPS-only
    { processGNHDT, "$GNHDT," }, // heading
    { processGPHDT, "$GPHDT," }, // heading, GPS-only
    { processGNZDA, "$GNZDA," }, // date and time
    { processGPZDA, "$GPZDA," }, // date and time, GPS-only
    { NULL, "" }
};


/**
 * @brief Initializes the serial port for communications with the CSBF GPS
 * @details No mutex protection, since it is assumed that only one thread
 * should be watching each serial port.
 * @param input_tty String holding the name of the comm port
 * @param verbosity How much should it talk to us?
 * @return int -1 on failure, file descriptor on success
 */
int CSBFsetSerial(const char *input_tty, int verbosity)
{
  int fd;
  struct termios term;

  if (verbosity > 0) blast_info("Connecting to sip port %s...", input_tty);

  if ((fd = open(input_tty, O_RDWR)) < 0) {
    if (verbosity > 0) {
        blast_err("Unable to open serial port");
    }
    return -1;
  }
  if (tcgetattr(fd, &term)) {
    if (verbosity > 0) {
        blast_err("Unable to get serial device attributes");
    }
    return -1;
  }

  term.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);
  term.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
  term.c_iflag |= INPCK;
  term.c_cc[VMIN] = 0;
  term.c_cc[VTIME] = 0;

  term.c_cflag &= ~(CSTOPB | CSIZE);
  term.c_oflag &= ~(OPOST);
  term.c_cflag |= CS8;

  if (cfsetospeed(&term, B19200)) {          /*  <======= SET THE SPEED HERE */
    if (verbosity > 0) blast_err("Error setting serial output speed");
    if (fd >= 0) close(fd);
    return -1;
  }

  if (cfsetispeed(&term, B19200)) {         /*  <======= SET THE SPEED HERE */
    if (verbosity > 0) blast_err("Error setting serial input speed");
    if (fd >= 0) close(fd);
    return -1;
  }

  if (tcsetattr(fd, TCSANOW, &term)) {
    if (verbosity > 0) blast_err("Unable to set serial attributes");
    if (fd >= 0) close(fd);
    return -1;
  }
  return fd;
}


/**
 * @brief performs a checksum on the GPS sentence
 * 
 * @param m_buf data buffer to check against
 * @param m_linelen size of the sentence to check against (seems iterated in use??)
 * @return true if checksum is good, false if checksum is bad
 */
static bool GPSverifyChecksum(const char *m_buf, size_t m_linelen)
{
    uint8_t checksum = 0;
    uint8_t recv_checksum = (uint8_t) strtol(&(m_buf[m_linelen - 2]), (char **)NULL, 16);

    // Start from index 1 because the first character ($) should not be included in the checksum.s
    for (size_t i = 1; i < m_linelen && m_buf[i] != '*'; i++) {
        checksum ^= m_buf[i];
    }
    if (recv_checksum != checksum) {
        blast_info("Received invalid checksum from NMEA GPS Data! For %s, expecting = %2x, we received %2x",
                   m_buf, checksum, recv_checksum);
        return false;
    }
    return true;
}


/**
 * @brief process a GGA formatted sentence - position & fix quality.
 * @details Access to the structs/variables updated by this function is
 * protected by mutex, since reception of GPS data from one or more channels
 * could cause this function to be called.
 * @param m_data data to process
 * @param fmt_str format string to parse a variant of a GGA sentence
 */
static void processGGA(const char* m_data, const char* fmt_str) {
    char lat_ns;
    char lon_ew;
    int lat, lon;
    double lat_mm;
    double lon_mm;
    float age_gps;
    static int first_time = 1;
    static int have_warned = 0;
    // blast_info("Starting process_gga");

    pthread_mutex_lock(&GGAlock);
    if (sscanf(m_data,
                fmt_str,
                &lat, &lat_mm, &lat_ns,
                &lon, &lon_mm, &lon_ew,
                &(CSBFGPSData.quality), &(CSBFGPSData.num_sat),
                &(CSBFGPSData.altitude), &age_gps) >= 9) {
        CSBFGPSData.latitude = (double)lat + lat_mm * GPS_MINS_TO_DEG;
        CSBFGPSData.longitude = (double)lon + lon_mm * GPS_MINS_TO_DEG;
        if ('S' == lat_ns) {
            CSBFGPSData.latitude *= -1.0;
        }
        if ('W' == lon_ew) {
            CSBFGPSData.longitude *= -1.0;
        }
        CSBFGPSData.isnew = 1;
        have_warned = 0;
        if (first_time) {
            blast_info("Recieved first packet:");
            blast_info("Read GGA: lat = %lf, lon = %lf, qual = %d, num_sat = %d, alt = %lf, age =%f",
                CSBFGPSData.latitude, CSBFGPSData.longitude, CSBFGPSData.quality,
                CSBFGPSData.num_sat, CSBFGPSData.altitude, age_gps);
            first_time = 0;
        }
    } else {
        if (!have_warned) {
            blast_info("Read GGA error. Sentence: %s", m_data);
            have_warned = 1;
        }
    }
    if (NMEA_CHATTER) {
        blast_info("%s", m_data);
    }
    pthread_mutex_unlock(&GGAlock);
}


/**
 * @brief process a GPGGA (US GPS) formatted position/fix sentence
 * 
 * @param m_data data to process
 */
static void processGPGGA(const char *m_data) {
    const char fmt_str[] = "$GPGGA,"
        "%*f,"       // UTC hhmmss.ss
        "%2d%lf,%c," // Latitude ddmm.mmmmm N/S
        "%3d%lf,%c," // Longitude dddmm.mmmmm E/W
        "%d,"        // GPS quality: 0 -> no fix, 1 -> gps fix, 2 differential fix
        "%d,"        // Number of satellites
        "%*f,"       // Horizontal Dilution of precision
        "%lf,M,"     // Altitude
        "%*f,M,"     // Geoidal separation
        "%f,"        // Age in seconds of GPS data
        "%*d,";      // Differential reference station ID
    processGGA(m_data, fmt_str);
}


/**
 * @brief process a GNGGA (GNSS) formatted position/fix sentence
 * 
 * @param m_data data to process
 */
static void processGNGGA(const char *m_data) {
    const char fmt_str[] = "$GNGGA,"
        "%*f,"       // UTC hhmmss.ss
        "%2d%lf,%c," // Latitude ddmm.mmmmm N/S
        "%3d%lf,%c," // Longitude dddmm.mmmmm E/W
        "%d,"        // GPS quality: 0 -> no fix, 1 -> gps fix, 2 differential fix
        "%d,"        // Number of satellites
        "%*f,"       // Horizontal Dilution of precision
        "%lf,M,"     // Altitude
        "%*f,M,"     // Geoidal separation
        "%f,"        // Age in seconds of GPS data
        "%*d,";      // Differential reference station ID
    processGGA(m_data, fmt_str);
}


/**
 * @brief process a HDT formatted sentence
 * @details Access to the structs/variables updated by this function is
 * protected by mutex, since reception of GPS data from one or more channels
 * could cause this function to be called.
 * @param m_data data to process
 * @param fmt_str format string to parse a variant of a HDT sentence
 */
static void processHDT(const char *m_data, const char* fmt_str)
{
    static int first_time = 1;
    static int have_warned = 0;
    double az_read = 0.0;
    size_t bytes_read = strnlen(m_data, 20);

    pthread_mutex_lock(&HDTlock);
    // Sometimes we don't get heading information.  mcp needs to be able to handle both cases.
    if (bytes_read < 14) {
        if (!have_warned) {
            blast_info("Not enough characters for HDT. We didn't get heading info. Sentence: %s", m_data);
            have_warned = 1;
            CSBFGPSAz.att_ok = 0;
        }
    } else {
        sscanf(m_data, fmt_str, &az_read);
        if ((az_read <= 0.0) || (az_read >= 360.0)) {
             // this almost certainly means we didn't read anything
            CSBFGPSAz.att_ok = 0;
        } else {
            CSBFGPSAz.att_ok = 1;
            CSBFGPSAz.az = az_read;
        }
        if (first_time) {
            blast_info("Read HDT heading = %lf", az_read);
            first_time = 0;
        }
    }
    if (NMEA_CHATTER) {
        blast_info("%s", m_data);
    }
    pthread_mutex_unlock(&HDTlock);
}


/**
 * @brief process a GPHDT (GPS) formatted position/fix sentence
 * 
 * @param m_data data to process
 */
static void processGPHDT(const char *m_data) {
    const char fmt_str[] = "$GPHDT,"
        "%lf,"  // Heading (deg) x.x
        "%*c,"; // True
    processHDT(m_data, fmt_str);
}


/**
 * @brief process a GNHDT (GNSS) formatted position/fix sentence
 * 
 * @param m_data data to process
 */
static void processGNHDT(const char *m_data) {
    const char fmt_str[] = "$GNHDT,"
        "%lf,"  // Heading (deg) x.x
        "%*c,"; // True
    processHDT(m_data, fmt_str);
}


/**
 * @brief process a ZDA formatted sentence - time info
 * @details Access to the structs/variables updated by this function is
 * protected by mutex, since reception of GPS data from one or more channels
 * could cause this function to be called.
 * @param m_data data to process
 * @param fmt_str format string to parse a variant of a ZDA sentence
 */
static void processZDA(const char* m_data, const char* fmt_str)
{
    static int first_time = 1;
    static int have_warned = 0;
    struct tm ts;
    // blast_info("Starting process_zda");

    pthread_mutex_lock(&ZDAlock);
    if (sscanf(m_data,
        fmt_str,
        &(ts.tm_hour),
        &(ts.tm_min),
        &(ts.tm_sec),
        &(ts.tm_mday),
        &(ts.tm_mon),
        &(ts.tm_year)) == 6) {
            ts.tm_year -= 1900;
            ts.tm_isdst = 0;
            ts.tm_mon--; /* Jan is 0 in struct tm.tm_mon, not 1 */
            csbf_gps_time = mktime(&ts);
            if (first_time) {
                blast_info("Read ZDA: hr = %2d, min = %2d, sec = %2d, mday = %d, mon = %d, year =%d",
                        ts.tm_hour, ts.tm_min, ts.tm_sec,
                        ts.tm_mday, ts.tm_mon, ts.tm_year);
                first_time = 0;
            }
    } else {
        if (!have_warned) {
            blast_err("Read ZDA error. Sentence: %s", m_data);
            have_warned = 1;
        }
    }
    if (NMEA_CHATTER) {
        blast_info("%s", m_data);
    }
    pthread_mutex_unlock(&ZDAlock);
}


/**
 * @brief process a GPZDA (GPS) formatted time sentence
 * 
 * @param m_data data to process
 */
static void processGPZDA(const char *m_data) {
    const char fmt_str[] = "$GPZDA,"
        "%2d%2d%2d.%*d,"
        "%d,"
        "%d,"
        "%d,"
        "%*d,"
        "%*d,";
    processZDA(m_data, fmt_str);
}


/**
 * @brief process a GNZDA (GNSS) formatted time sentence
 * 
 * @param m_data data to process
 */
static void processGNZDA(const char *m_data) {
    const char fmt_str[] = "$GNZDA,"
        "%2d%2d%2d.%*d,"
        "%d,"
        "%d,"
        "%d,"
        "%*d,"
        "%*d,";
    processZDA(m_data, fmt_str);
}


/**
 * @brief thread function to monitor the DGPS serial port and deal with
 * receiving and decrypting the data.
 * @param arg unused
 * @return void* threads require typing void *
 */
void* DGPSmonitorSerial(void * arg)
{
    char tname[18];
    int get_serial_fd = 1;
    int tty_fd;
    int timer = 0;
    unsigned char buf;
    char indata[128];
    uint16_t i_char = 0;
    static int has_warned = 0;
    static int first_time = 1;
    e_dgps_read_status readstage = DGPS_WAIT_FOR_START;

    snprintf(tname, sizeof(tname), "DGPSmonitorSerial");
    nameThread(tname);
    blast_startup("Starting DGPSmonitorSerial thread.");
    for (;;) {
        // usleep(10000); /* sleep for 10ms */
        // wait for a valid file descriptor
        while (get_serial_fd) {
            if ((tty_fd = CSBFsetSerial(CSBFGPSCOM, !has_warned)) >= 0) {
                break;
            }
            has_warned = 1;
            sleep(5);
        }
        has_warned = 0;
        get_serial_fd = 0;
        /* Loop until data come in */
        while (read(tty_fd, &buf, 1) <= 0) {
            usleep(10000);
            timer++;
        }
        if (get_serial_fd) {
            break;
        }
        if (buf == '$') {
            readstage = DGPS_READING_PKT;
            i_char = 0;
        }
        if (DGPS_WAIT_FOR_START == readstage) {
            continue; // still haven't found start byte
        }
        if (i_char >= 128) {
            blast_err("Read from DGPS a packet longer than the buffer. %s", indata);
            readstage = DGPS_WAIT_FOR_START;
            continue;
        }
        indata[i_char] = buf;
        if (buf == '\r') {
            indata[i_char] = '\0'; // Terminate with '\0' instead of '\r'
            if (first_time) {
                blast_info("Finished reading packet %s", indata);
            }
            if (!GPSverifyChecksum(indata, i_char)) {
                blast_err("checksum failed");
            } else {
                if (NMEA_CHATTER) {
                    blast_info("Parsing NMEA sentence from serial...");
                }
                // Select any handlers that match the sentence
                for (nmea_handler_t *handler = handlers; handler->proc; handler++) {
                    if (!strncmp(indata, handler->str, (int) strlen(handler->str)-1)) {
                        handler->proc(indata);
                    }
                }
            }
            if (first_time) {
                blast_info("Finished calling the handlers");
                first_time = 0;
            }
        }
        i_char++;
    }
    return NULL;
}


/**
 * @brief thread function to monitor the DGPS UDP port and deal with
 * receiving and decrypting the data.
 * @details The GPS module must be configured to send UDP packets to the
 * IP address of each flight computer to be received here. Receivers can
 * ususally be configured to send at a cadence - we expect 1 Hz.
 * @param arg unused
 * @return void* threads require typing void *
 */
void* DGPSmonitorUDP(void* args) {
    struct socketData* socket_target = args;

    char tname[15];
    int err;
    int first_time = 1;
    int numbytes;
    int returnval;
    int sockfd;

    char ipAddr[INET_ADDRSTRLEN];

    struct addrinfo hints;
    struct addrinfo *servinfo;
    struct addrinfo *servinfoCheck = NULL;
    struct sockaddr_storage sender_addr;

    socklen_t addr_len;

    memset(&hints, 0, sizeof(hints));

    hints.ai_family = AF_INET; // set to AF_INET to use IPv4
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_flags = AI_PASSIVE; // use my IP

    char nmea_buffer[MAXLEN_NMEA_0183_MESSAGE];

    snprintf(tname, sizeof(tname), "DGPSmonitorUDP");
    nameThread(tname);
    blast_startup("Starting DGPSmonitorUDP thread.");

    while (1) {
        if (first_time) {
            first_time = 0;
            if ((returnval = getaddrinfo(NULL, socket_target->port, &hints, &servinfo)) != 0) {
                blast_info("getaddrinfo: %s\n", gai_strerror(returnval));
                return NULL;
            }
            // loop through all the results and bind to the first we can
            for (servinfoCheck = servinfo; servinfoCheck != NULL; servinfoCheck = servinfoCheck->ai_next) {
                // since a success needs both of these, but it is inefficient to do fails twice,
                // continue skips the second if on a fail
                if ((sockfd = socket(servinfoCheck->ai_family, servinfoCheck->ai_socktype,
                        servinfoCheck->ai_protocol)) == -1) {
                    blast_err("Failed to make socket");
                    continue;
                }
                if (bind(sockfd, servinfoCheck->ai_addr, servinfoCheck->ai_addrlen) == -1) {
                    close(sockfd);
                    blast_err("Failed to bind to socket");
                    continue;
                }
                break;
            }
            if (servinfoCheck == NULL) {
                blast_err("Failed to find any sockets");
                return NULL;
            }
            // set the read timeout (if there isn't a message)
            struct timeval read_timeout;
            int read_timeout_usec = 1000;
            read_timeout.tv_sec = 0;
            read_timeout.tv_usec = read_timeout_usec;
            setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR | SO_RCVTIMEO, &read_timeout, sizeof(read_timeout));
            // now we set up the print statement vars
            // need to cast the socket address to an INET still address
            struct sockaddr_in *ipv = (struct sockaddr_in *)servinfo->ai_addr;
            // then pass the pointer to translation and put it in a string
            inet_ntop(AF_INET, &(ipv->sin_addr), ipAddr, INET_ADDRSTRLEN);
        }
        numbytes = recvfrom(
            sockfd,
            nmea_buffer,
            sizeof(nmea_buffer),
            0,
            (struct sockaddr *) &sender_addr,
            &addr_len);
        // we get an error everytime it times out, but EAGAIN is ok, other ones are bad.
        if (numbytes == -1) {
            err = errno;
            if (err != EAGAIN) {
                blast_info("Errno is %d\n", err);
                blast_info("Error is %s\n", strerror(err));
                berror(err, "Recvfrom");
            }
        } else {
            // Tokenize str from UDP packet: the GPS receiver we talk to
            // stuffs all NMEA sentences into a single packet, so we separate
            // them here
            char* pTok = NULL;
            char* pSave = NULL;
            pTok = strtok_r(nmea_buffer, "\r\n", &pSave);
            while (pTok != NULL) {
                if (!GPSverifyChecksum(pTok, strlen(pTok))) {
                    blast_err("checksum failed");
                } else {
                    if (NMEA_CHATTER) {
                        blast_info("Parsing NMEA sentence from UDP...");
                    }
                    // Select any handlers that match the sentence
                    for (nmea_handler_t *handler = handlers; handler->proc; handler++) {
                        if (!strncmp(pTok, handler->str, (int) strlen(handler->str) - 1)) {
                            handler->proc(pTok);
                        }
                    }
                }
                pTok = strtok_r(NULL, "\r\n", &pSave);
            }
        }
    }
    freeaddrinfo(servinfo);
    close(sockfd);
    return NULL;
}

/**
 * @brief Multiple methods of receiving NMEA-formatted GNSS data run
 * concurrently.
 * @details NMEA messages are parsed from a dedicated serial port and UDP
 * packets. Access to the data used by the rest of the program is mutexed,
 * to avoid race conditions when serial/UDP messages arrive simultaneously.
 * As a result, the last message that comes in "wins".
 */
void StartDGPSmonitors(void)
{
    // Dedicated serial port
    pthread_t DGPSserialThread;
    pthread_create(&DGPSserialThread, NULL, DGPSmonitorSerial, NULL);
    pthread_detach(DGPSserialThread);

    // UDP from a specific IP address and port - set in GPS unit webserver
    // settings menu
    pthread_t DGPSudpThread;
    struct socketData socket_data;
    if (populateSocketData(GPS_IP_ADDR, GPS_PORT, &socket_data) < 0) {
        blast_info("Failed to populate socket data, cannot receive NMEA sentences via UDP");
    } else {
        pthread_create(&DGPSudpThread, NULL, DGPSmonitorUDP, (void *) &socket_data);
        pthread_detach(DGPSudpThread);
    }
}
