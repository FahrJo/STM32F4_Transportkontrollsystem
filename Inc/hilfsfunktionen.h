/**
   ----------------------------------------------------------------------
   frei nach ... bearbeitet
	Copyright (c) 2016 Tilen Majerle

    Permission is hereby granted, free of charge, to any person
    obtaining a copy of this software and associated documentation
    files (the "Software"), to deal in the Software without restriction,
    including without limitation the rights to use, copy, modify, merge,
    publish, distribute, sublicense, and/or sell copies of the Software, 
    and to permit persons to whom the Software is furnished to do so, 
    subject to the following conditions:

    The above copyright notice and this permission notice shall be
    included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
    OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
    AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
    OTHER DEALINGS IN THE SOFTWARE.
   ----------------------------------------------------------------------
 */
#ifndef hilfsfunktionen_H
#define hilfsfunktionen_H 020

/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif

/**
 * \defgroup GPS
 * \brief  Platform independent, written in ANSII C, GPS AT parser library for SIMcom modules   
 * \{
 ?
 */
#include "stdlib.h"
#include "string.h"
#include "stdint.h"
#include "stdio.h"
#include "math.h"

/* Buffer implementation */
//#include "buffer.h"


/* Parses and returns number from string */
int32_t myParseNumber(const char* ptr, uint8_t* cnt);

/* Parse HEX number */
uint32_t myParseHexNumber(const char* ptr, uint8_t* cnt);

/* Parse float number */
float myParseFloatNumber(const char* ptr, uint8_t* cnt);


/* C++ detection */
#ifdef __cplusplus
}
#endif

#endif
