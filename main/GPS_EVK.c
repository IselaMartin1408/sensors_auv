#include "GPS_EVK.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

static char sentence[128];
static int idx = 0;

void gps_init(gps_data_t *gps) {
    memset(gps, 0, sizeof(gps_data_t));
}

static double nmea_to_deg(const char *val, const char *dir) {
    double raw = atof(val);
    int degrees = (int)(raw / 100);
    double minutes = raw - (degrees * 100);
    double deg = degrees + minutes / 60.0;

    if (dir[0] == 'S' || dir[0] == 'W')
        deg *= -1.0;

    return deg;
}

void gps_parse_sentence(gps_data_t *gps, char *s) {
    if (strstr(s, "GGA")) {
        // Ejemplo: $GPGGA,170834,4124.8963,N,08151.6838,W,1,05,1.5,280.2,M,...
        char *tok = strtok(s, ",");
        int field = 0;
        char *dir_tok = NULL;
        
        while (tok && field < 15) {  // Limit field count to prevent overflow
            switch (field) {
                case 2: 
                    dir_tok = strtok(NULL, ",");
                    if (dir_tok) {
                        gps->latitude = nmea_to_deg(tok, dir_tok);
                        field++;  // Skip next field (direction already consumed)
                    }
                    break;
                case 4: 
                    dir_tok = strtok(NULL, ",");
                    if (dir_tok) {
                        gps->longitude = nmea_to_deg(tok, dir_tok);
                        field++;  // Skip next field
                    }
                    break;
                case 6: 
                    if (atoi(tok) > 0) gps->valid_fix = true;
                    break;
                case 7: 
                    gps->satellites = atoi(tok); 
                    break;
                case 9: 
                    gps->altitude = atof(tok); 
                    break;
            }
            tok = strtok(NULL, ",");
            field++;
        }
    }

    if (strstr(s, "RMC")) {
        // Ejemplo: $GPRMC,130059.00,A,2503.7120,N,12138.7459,E,...
        char *tok = strtok(s, ",");
        int field = 0;
        char *dir_tok = NULL;

        while (tok && field < 15) {  // Safety limit
            switch (field) {
                case 1: // hora - validate length
                    if (strlen(tok) >= 6) {
                        gps->hour = (tok[0]-'0')*10 + (tok[1]-'0');
                        gps->minute = (tok[2]-'0')*10 + (tok[3]-'0');
                        gps->second = (tok[4]-'0')*10 + (tok[5]-'0');
                    }
                    break;

                case 3: 
                    dir_tok = strtok(NULL, ",");
                    if (dir_tok) {
                        gps->latitude = nmea_to_deg(tok, dir_tok);
                        field++;  // Skip direction field
                    }
                    break;
                    
                case 5: 
                    dir_tok = strtok(NULL, ",");
                    if (dir_tok) {
                        gps->longitude = nmea_to_deg(tok, dir_tok);
                        field++;  // Skip direction field
                    }
                    break;
                    
                case 7: 
                    gps->speed_kmph = atof(tok) * 1.852; 
                    break;
                    
                case 8: 
                    gps->course_deg = atof(tok); 
                    break;

                case 9: // fecha ddmmyy - validate length
                    if (strlen(tok) >= 6) {
                        gps->day   = (tok[0]-'0')*10 + (tok[1]-'0');
                        gps->month = (tok[2]-'0')*10 + (tok[3]-'0');
                        gps->year  = 2000 + (tok[4]-'0')*10 + (tok[5]-'0');
                    }
                    break;
            }
            tok = strtok(NULL, ",");
            field++;
        }
    }
}

void gps_encode(gps_data_t *gps, char c) {
    if (c == '$') {
        idx = 0;
        memset(sentence, 0, sizeof(sentence));
    }

    if (idx < sizeof(sentence)-1) {
        sentence[idx++] = c;
    }

    // Parse on newline or carriage return
    if (c == '\n' || c == '\r') {
        if (idx > 10) {  // Valid NMEA must be at least this long
            sentence[idx] = '\0';  // Null terminate
            gps_parse_sentence(gps, sentence);
        }
        idx = 0;
    }
}
