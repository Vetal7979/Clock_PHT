#include "main.h"

#include "gps.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

extern uint16_t date_set_timeout; //, timeout_2, timeout_wifi, timeout_1pps;

#define GPS_BUFFER_SIZE 1024 // 150
char nmea[GPS_BUFFER_SIZE]; 
uint8_t nmea_index = 0, start_sentence = 0;

char nmea2[GPS_BUFFER_SIZE];
uint8_t nmea_index2 = 0, start_sentence2 = 0;

extern uint8_t psim_ok,psim_ok2;

#define bool u8
#define false 0
#define true 1

struct tm gps_time = { 0 }, gps_time2 = { 0 };
struct tm ttime,ttime2;

volatile char time_set = false, date_set = false, gps_valid = false;
volatile char time_set2 = false, date_set2 = false, gps_valid2 = false;
u8 GGA_flag = 0;
u8 GGA_flag2 = 0;

long lat_prev = 0, lon_prev = 0, alt_prev = 0, course_prev = 0, speed_prev = 0, dt_prev = 0;
long lat_prev2 = 0, lon_prev2 = 0, alt_prev2 = 0, course_prev2 = 0, speed_prev2 = 0, dt_prev2 = 0;
u8 has_date = 0;
u8 has_date2 = 0;
unsigned long last_dt = 0;
unsigned long last_dt2 = 0;

struct Controller_ControlFrame current = { 0, 0, 0, 0, 0, 0 };
struct Controller_ControlFrame current2 = { 0, 0, 0, 0, 0, 0 };
int recs = 0;
int recs2 = 0;

 int atoin(const char* s, int n) {
    char buf[10];
    memcpy(buf, s, n);
    buf[n] = '\0';
    return atoi(buf);
}

long my_abs(long a) {
    return (a > 0 ? a : -a);
}

int checksum() {
    int cs = 0;
    for (int i = 1; nmea[i] && nmea[i] != '*'; i ++) {
        cs = cs ^ (signed char)nmea[i];
    }
    return cs;
}

u8 valid_coord() {
    u8 valid = (
        current.lat >= -180000000L && current.lat <= 180000000L &&
        current.lon >= -180000000L && current.lon <= 180000000L &&
        current.dt >= 1262304000L && current.dt <= 2524608000L // 2010-01-01 .. 2050-01-01
	);

    return valid;
}


int get_token(char* data, int index, char** out) {
    // strtok can`t handle multiple delims in a row (like ,,,,, in "GNRMC,210842.086,V,,,,,0.92,269.72,010718,,,N")
    const char delim = ',';
    unsigned int start = 0;
    unsigned int pos = 0;
    while (true){
        char c = data[pos++];
        if (c == 0) break;
        if (c == delim || c == '*'){
            if (index == 0){ // needed delim found
                *out = data+start;
                return pos - start -1;
            }
            if (index == 1){ // save start here
                start = pos;
            }
            index--;
        }
    }
    return 0;
}

char parse_packet_nmea(char *_nmea[], bool isGGA) {
    char flushed = 0;

    char *time0 = _nmea[1];
    char *slat  = _nmea[isGGA ? 2 : 3];
    bool isN    = ((*_nmea[isGGA ? 3 : 4]) == 'N');
    char *slon  = _nmea[isGGA ? 4 : 5];
    bool isE    = ((*_nmea[isGGA ? 5 : 6]) == 'E');

    double lat_start = (double)atoin(slat, 2);
    double lat_end   = (double)atof(slat + 2) / 60;
    double _lat      = lat_start + lat_end;

    double lon_start = (double)atoin(slon, 3);
    double lon_end   = (double)atof(slon + 3) / 60;
    double _lon      = lon_start + lon_end;

    ttime.tm_hour = atoin(time0 + 0, 2);
    ttime.tm_min  = atoin(time0 + 2, 2);
    ttime.tm_sec  = atoin(time0 + 4, 2);

    double xlat = _lat * (isN ? 1 : -1);
    double xlon = _lon * (isE ? 1 : -1);

    current.lat = xlat * 1000000;
    current.lon = xlon * 1000000;

    current.dt = mktime(&ttime);

    if (valid_coord()) {
        int delta = (int)(current.dt - dt_prev);

        if (delta == 0) {
            return 2; // same coordinate
        }

        if (lat_prev == 0 || (dt_prev != 0 && (delta > 1))) { // first coord or delta(dt) > 1
           // if (buffer_offset > SECTOR_SIZE - sizeof(struct Controller_ControlFrame)) { // sector overflow
                //flush_sector();
               // flushed = 1;
           // }

           // buffer_offset += controller_control_frame(&sector_buffer[buffer_offset]);
           // set_bit(&ch.hdr[0], 1);
        } else {
            long lat_delta    = (current.lat    - lat_prev),
                 lon_delta    = (current.lon    - lon_prev),
                 alt_delta    = (current.alt    - alt_prev),
                 course_delta = (current.course - course_prev),
                 speed_delta  = (current.speed  - speed_prev);
            if (my_abs(lat_delta) > 127 || my_abs(lon_delta) > 127 || my_abs(alt_delta) > 127 || my_abs(course_delta) > 127 || my_abs(speed_delta) > 127) { // deltas overflow
               // if (buffer_offset > SECTOR_SIZE - sizeof(struct Controller_ControlFrame)) { // sector overflow
                    //flush_sector();
                   // flushed = 1;
              //  }

               // buffer_offset += controller_control_frame(&sector_buffer[buffer_offset]);
               // set_bit(&ch.hdr[0], 1);
           // } else {
               // if (buffer_offset > SECTOR_SIZE - sizeof(struct Controller_DiffFrame)) { // sector overflow
                    //flush_sector();
                   // flushed = 1;

                   // buffer_offset += controller_control_frame(&sector_buffer[buffer_offset]);
                   // set_bit(&ch.hdr[0], 1);
                //} else {
                  //  buffer_offset += controller_diff_frame(&sector_buffer[buffer_offset], (int)lat_delta, (int)lon_delta, (int)alt_delta, (int)course_delta, (int)speed_delta);
                  //  set_bit(&ch.hdr[0], 0);
                //}
            }
        }

        lat_prev    = current.lat;
        lon_prev    = current.lon;
        alt_prev    = current.alt;
        course_prev = current.course;
        speed_prev  = current.speed;
        dt_prev     = current.dt;
    } else {
    	return 3; // invalid coordinate
    }

    return flushed;
}

void parse_nmea() {
    int len;
    char *token;

    if (strstr(nmea, "GGA") != NULL) {

      GGA_flag = 1;

        len = get_token(nmea, 1, &token);
        if (len == 10 || len == 9) {
            gps_time.tm_hour = atoin(token + 0, 2);
            gps_time.tm_min  = atoin(token + 2, 2);
            gps_time.tm_sec  = atoin(token + 4, 2);
            
            time_set = true; 

            len = get_token(nmea, 4, &token);
            if (len == 1) {
                gps_valid = (token[0] == 'N' || token[0] == 'S');
            }
        }
    } else {
        len = get_token(nmea, 1, &token);
        if (len == 10 || len == 9) {
            gps_time.tm_hour = atoin(token + 0, 2);
            gps_time.tm_min  = atoin(token + 2, 2);
            gps_time.tm_sec  = atoin(token + 4, 2);

            time_set = true; 

            len = get_token(nmea, 9, &token);
            if (len == 6) {
                gps_time.tm_mday  = atoin(token + 0, 2);
                gps_time.tm_mon   = atoin(token + 2, 2);
                gps_time.tm_year  = atoin(token + 4, 2);
                if (gps_time.tm_year >= 18 && gps_time.tm_year <= 55) {
                    date_set = true; date_set_timeout = 0;

                    len = get_token(nmea, 2, &token);
                    if (len == 1) {
                        gps_valid = (token[0] == 'A');

                    }
                }
            }
        }
    }
}

  
 void gps_received(uint8_t buffer) {

        if (buffer == '$') {
            start_sentence = 1;
            nmea_index = 0;
        } else if (buffer == '\r' || buffer == '\n') {
            if ((strncmp((const char *)nmea, "$GPRMC", 6) == 0 || strncmp((const char *)nmea, "$GNRMC", 6) == 0) ||
                (strncmp((const char *)nmea, "$GPGGA", 6) == 0 || strncmp((const char *)nmea, "$GNGGA", 6) == 0) ||
                (strncmp((const char *)nmea, "$GLRMC", 6) == 0 || strncmp((const char *)nmea, "$GLGGA", 6) == 0)) {
               //printf("%s \r\n",nmea);
                  parse_nmea();
            }
            else
           if (strncmp((const char *)nmea, "$PSIMPPS,W,Ok*27", 16) == 0) 
             { psim_ok=1; }

            start_sentence = 0;
        }
        if (start_sentence) {
            nmea[nmea_index] = buffer;
            nmea[nmea_index + 1] = 0;
            nmea_index ++;
            if (nmea_index >= sizeof(nmea) / sizeof(nmea[0]) - 1) {
                start_sentence = 0;
            }
        }

}

//------------------------------------------------------------------------------
int checksum2() {
    int cs = 0;
    for (int i = 1; nmea2[i] && nmea2[i] != '*'; i ++) {
        cs = cs ^ (signed char)nmea2[i];
    }
    return cs;
}

u8 valid_coord2() {
    u8 valid = (
        current2.lat >= -180000000L && current2.lat <= 180000000L &&
        current2.lon >= -180000000L && current2.lon <= 180000000L &&
        current2.dt >= 1262304000L && current2.dt <= 2524608000L // 2010-01-01 .. 2050-01-01
	);

    return valid;
}


int get_token2(char* data, int index, char** out) {
    // strtok can`t handle multiple delims in a row (like ,,,,, in "GNRMC,210842.086,V,,,,,0.92,269.72,010718,,,N")
    const char delim = ',';
    unsigned int start = 0;
    unsigned int pos = 0;
    while (true){
        char c = data[pos++];
        if (c == 0) break;
        if (c == delim || c == '*'){
            if (index == 0){ // needed delim found
                *out = data+start;
                return pos - start -1;
            }
            if (index == 1){ // save start here
                start = pos;
            }
            index--;
        }
    }
    return 0;
}

char parse_packet_nmea2(char *_nmea[], bool isGGA) {
    char flushed = 0;

    char *time0 = _nmea[1];
    char *slat  = _nmea[isGGA ? 2 : 3];
    bool isN    = ((*_nmea[isGGA ? 3 : 4]) == 'N');
    char *slon  = _nmea[isGGA ? 4 : 5];
    bool isE    = ((*_nmea[isGGA ? 5 : 6]) == 'E');

    double lat_start = (double)atoin(slat, 2);
    double lat_end   = (double)atof(slat + 2) / 60;
    double _lat      = lat_start + lat_end;

    double lon_start = (double)atoin(slon, 3);
    double lon_end   = (double)atof(slon + 3) / 60;
    double _lon      = lon_start + lon_end;

    ttime2.tm_hour = atoin(time0 + 0, 2);
    ttime2.tm_min  = atoin(time0 + 2, 2);
    ttime2.tm_sec  = atoin(time0 + 4, 2);

    double xlat = _lat * (isN ? 1 : -1);
    double xlon = _lon * (isE ? 1 : -1);

    current2.lat = xlat * 1000000;
    current2.lon = xlon * 1000000;

    current2.dt = mktime(&ttime2);

    if (valid_coord2()) {
        int delta = (int)(current2.dt - dt_prev2);

        if (delta == 0) {
            return 2; // same coordinate
        }

        if (lat_prev2 == 0 || (dt_prev2 != 0 && (delta > 1))) { // first coord or delta(dt) > 1
           // if (buffer_offset > SECTOR_SIZE - sizeof(struct Controller_ControlFrame)) { // sector overflow
                //flush_sector();
               // flushed = 1;
           // }

           // buffer_offset += controller_control_frame(&sector_buffer[buffer_offset]);
           // set_bit(&ch.hdr[0], 1);
        } else {
            long lat_delta2    = (current2.lat    - lat_prev2),
                 lon_delta2    = (current2.lon    - lon_prev2),
                 alt_delta2    = (current2.alt    - alt_prev2),
                 course_delta2 = (current2.course - course_prev2),
                 speed_delta2  = (current2.speed  - speed_prev2);
            if (my_abs(lat_delta2) > 127 || my_abs(lon_delta2) > 127 || my_abs(alt_delta2) > 127 || my_abs(course_delta2) > 127 || my_abs(speed_delta2) > 127) { // deltas overflow
               // if (buffer_offset > SECTOR_SIZE - sizeof(struct Controller_ControlFrame)) { // sector overflow
                    //flush_sector();
                   // flushed = 1;
              //  }

               // buffer_offset += controller_control_frame(&sector_buffer[buffer_offset]);
               // set_bit(&ch.hdr[0], 1);
           // } else {
               // if (buffer_offset > SECTOR_SIZE - sizeof(struct Controller_DiffFrame)) { // sector overflow
                    //flush_sector();
                   // flushed = 1;

                   // buffer_offset += controller_control_frame(&sector_buffer[buffer_offset]);
                   // set_bit(&ch.hdr[0], 1);
                //} else {
                  //  buffer_offset += controller_diff_frame(&sector_buffer[buffer_offset], (int)lat_delta, (int)lon_delta, (int)alt_delta, (int)course_delta, (int)speed_delta);
                  //  set_bit(&ch.hdr[0], 0);
                //}
            }
        }

        lat_prev2    = current2.lat;
        lon_prev2    = current2.lon;
        alt_prev2    = current2.alt;
        course_prev2 = current2.course;
        speed_prev2  = current2.speed;
        dt_prev2     = current2.dt;
    } else {
    	return 3; // invalid coordinate
    }

    return flushed;
}

void parse_nmea2() {
    int len;
    char *token;

    if (strstr(nmea2, "GGA") != NULL) {

      GGA_flag2 = 1;

        len = get_token2(nmea2, 1, &token);
        if (len == 10 || len == 9) {
            gps_time2.tm_hour = atoin(token + 0, 2);
            gps_time2.tm_min  = atoin(token + 2, 2);
            gps_time2.tm_sec  = atoin(token + 4, 2);
            
            time_set2 = true;

            len = get_token2(nmea2, 4, &token);
            if (len == 1) {
                gps_valid2 = (token[0] == 'N' || token[0] == 'S');
            }
        }
    } else {
        len = get_token2(nmea2, 1, &token);
        if (len == 10 || len == 9) {
            gps_time2.tm_hour = atoin(token + 0, 2);
            gps_time2.tm_min  = atoin(token + 2, 2);
            gps_time2.tm_sec  = atoin(token + 4, 2);

            time_set2 = true;

            len = get_token2(nmea2, 9, &token);
            if (len == 6) {
                gps_time2.tm_mday  = atoin(token + 0, 2);
                gps_time2.tm_mon   = atoin(token + 2, 2);
                gps_time2.tm_year  = atoin(token + 4, 2);
                if (gps_time2.tm_year >= 18 && gps_time2.tm_year <= 55) {
                    date_set2 = true; /*timeout_2 = 0;*/

                    len = get_token2(nmea2, 2, &token);
                    if (len == 1) {
                        gps_valid2 = (token[0] == 'A');

                    }
                }
            }
        }
    }
}

  
 void gps_received2(uint8_t buffer) {

        if (buffer == '$') {
            start_sentence2 = 1;
            nmea_index2 = 0;
        } else if (buffer == '\r' || buffer == '\n') {
            if ((strncmp((const char *)nmea2, "$GPRMC", 6) == 0 || strncmp((const char *)nmea2, "$GNRMC", 6) == 0) ||
                (strncmp((const char *)nmea2, "$GPGGA", 6) == 0 || strncmp((const char *)nmea2, "$GNGGA", 6) == 0) ||
                (strncmp((const char *)nmea2, "$GLRMC", 6) == 0 || strncmp((const char *)nmea2, "$GLGGA", 6) == 0)) {
               //printf("%s \r\n",nmea);
                  parse_nmea2();
            }
            else
           if (strncmp((const char *)nmea2, "$PSIMPPS,W,Ok*27", 16) == 0) { psim_ok2=1; }

            start_sentence2 = 0;
        }
        if (start_sentence2) {
            nmea2[nmea_index2] = buffer;
            nmea2[nmea_index2 + 1] = 0;
            nmea_index2 ++;
            if (nmea_index2 >= sizeof(nmea2) / sizeof(nmea2[0]) - 1) {
                start_sentence2 = 0;
            }
        }

}