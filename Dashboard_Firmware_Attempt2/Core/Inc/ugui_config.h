/*
 * ugui_config.h
 *
 *  Created on: Dec 4, 2023
 *      Author: leoja
 */

#ifndef INC_UGUI_CONFIG_H_
#define INC_UGUI_CONFIG_H_

#include "stdint.h"

/* -------------------------------------------------------------------------------- */
/* -- CONFIG SECTION                                                             -- */
/* -------------------------------------------------------------------------------- */


/* Enable color mode */
//#define USE_COLOR_RGB888   // RGB = 0xFF,0xFF,0xFF
#define USE_COLOR_RGB565

/* Enable needed fonts here */
//#define  USE_FONT_4X6
//#define  USE_FONT_5X8
//#define  USE_FONT_5X12
//#define  USE_FONT_6X8
//#define  USE_FONT_6X10
//#define  USE_FONT_7X12
//#define  USE_FONT_8X8
//#define  USE_FONT_8X12_CYRILLIC
//#define  USE_FONT_8X12
//#define  USE_FONT_8X12
//#define  USE_FONT_8X14
//#define  USE_FONT_10X16
#define  USE_FONT_12X16
//#define  USE_FONT_12X20
#define  USE_FONT_16X26
//#define  USE_FONT_22X36
#define  USE_FONT_24X40
#define  USE_FONT_32X53

/* Specify platform-dependent integer types here */

#define __UG_FONT_DATA const
typedef uint8_t      UG_U8;
typedef int8_t       UG_S8;
typedef uint16_t     UG_U16;
typedef int16_t      UG_S16;
typedef uint32_t     UG_U32;
typedef int32_t      UG_S32;


/* -------------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------------- */


/* Feature enablers */
#define USE_PRERENDER_EVENT
#define USE_POSTRENDER_EVENT


/* -----------------------------     End of File      ----------------------------- */


#endif /* INC_UGUI_CONFIG_H_ */
