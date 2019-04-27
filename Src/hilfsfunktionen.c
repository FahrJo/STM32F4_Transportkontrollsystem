/**
* |----------------------------------------------------------------------
* | Frei nach dieser Vorlage von
* | Copyright (c) 2016 Tilen Majerle
* |
* | Permission is hereby granted, free of charge, to any person
* | obtaining a copy of this software and associated documentation
* | files (the "Software"), to deal in the Software without restriction,
* | including without limitation the rights to use, copy, modify, merge,
* | publish, distribute, sublicense, and/or sell copies of the Software,
* | and to permit persons to whom the Software is furnished to do so,
* | subject to the following conditions:
* |
* | The above copyright notice and this permission notice shall be
* | included in all copies or substantial portions of the Software.
* |
* | THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* | EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
* | OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
* | AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
* | HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* | WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* | FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* | OTHER DEALINGS IN THE SOFTWARE.
* |----------------------------------------------------------------------
*/


#include "hilfsfunktionen.h"

 

#define CHARISNUM(x)                        ((x) >= '0' && (x) <= '9')
#define CHARISHEXNUM(x)                     (((x) >= '0' && (x) <= '9') || ((x) >= 'a' && (x) <= 'f') || ((x) >= 'A' && (x) <= 'F'))
#define CHARTONUM(x)                        ((x) - '0')
#define CHARHEXTONUM(x)                     (((x) >= '0' && (x) <= '9') ? ((x) - '0') : (((x) >= 'a' && (x) <= 'z') ? ((x) - 'a' + 10) : (((x) >= 'A' && (x) <= 'Z') ? ((x) - 'A' + 10) : 0)))
#define FROMMEM(x)                          ((const char *)(x))

#define GPS_ADDTOCRC(ch)                    do { Int.CRC ^= (uint8_t)(ch); } while (0)
#define GPS_ADDTOTERM(ch)                   do { Int.Term[Int.Flags.F.Term_Pos++] = (ch); Int.Term[Int.Flags.F.Term_Pos] = 0; } while (0);    /* Add new element to term object */
#define GPS_START_NEXT_TERM()               do { Int.Term[0] = 0; Int.Flags.F.Term_Pos = 0; Int.Flags.F.Term_Num++; } while (0);

#define GPS_EARTH_RADIUS                    6371.0f            /* Earth radius */
#define GPS_DEGREES2RADIANS(x)              (float)((x) * 0.01745329251994f)   /* Degrees to radians */
#define GPS_RADIANS2DEGREES(x)              (float)((x) * 57.29577951308232f)  /* Radians to degrees */
#define GPS_MAX_SATS_IN_VIEW                24                  /* Maximal number of satellites in view */

/* GPS statements definitions */
#define GPS_UNKNOWN                         0
#define GPS_GPGGA                           1
#define GPS_GPGSA                           2
#define GPS_GPGSV                           3
#define GPS_GPRMC                           4
#define GPS_FLAGS_ALL                       (1 << GPS_GPGGA | 1 << GPS_GPGSA | 1 << GPS_GPGSV | 1 << GPS_GPRMC)

#define GPS_CONCAT(x, y)                    (uint16_t)((x) << 8 | (y))


/**************************************************************************
***************************************************************************
***************************************************************************
**************************************************************************/

/* Parses and returns number from string */
int32_t myParseNumber(const char* ptr, uint8_t* cnt){
    uint8_t minus = 0, i = 0;
    int32_t sum = 0;

    if (*ptr == '-') {                                      /* Check for minus character */
        minus = 1;
        ptr++;
        i++;
    }
    while (CHARISNUM(*ptr)) {                               /* Parse number */
        sum = 10 * sum + CHARTONUM(*ptr);
        ptr++;
        i++;
    }
    if (cnt != NULL) {                                      /* Save number of characters used for number */
        *cnt = i;
    }
    if (minus) {                                            /* Minus detected */
        return -sum;
    }
    return sum;                                             /* Return number */
}

/* Parse HEX number */
uint32_t myParseHexNumber(const char* ptr, uint8_t* cnt) {
    uint8_t i = 0;
    uint32_t sum = 0;

    while (CHARISHEXNUM(*ptr)) {                            /* Parse number */
        sum = 16 * sum + CHARHEXTONUM(*ptr);
        ptr++;
        i++;
    }
    if (cnt != NULL) {                                      /* Save number of characters used for number */
        *cnt = i;
    }
    return sum;                                             /* Return number */
}

/* Parse float number */
float myParseFloatNumber(const char* ptr, uint8_t* cnt) {
    uint8_t i = 0, j = 0;
    float sum = 0.0f;

    sum = (float)myParseNumber(ptr, &i);                      /* Parse number */
    j += i;
    ptr += i;
    if (*ptr == '.') {                                      /* Check decimals */
        float dec;
        dec = (float)myParseNumber(ptr + 1, &i) / (float)pow(10, i);
        if (sum >= 0) {
            sum += dec;
        } else {
            sum -= dec;
        }
        j += i + 1;
    }

    if (cnt != NULL) {                                      /* Save number of characters used for number */
        *cnt = j;
    }
    return sum;                                             /* Return number */
}

