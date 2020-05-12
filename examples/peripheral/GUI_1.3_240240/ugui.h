/* -------------------------------------------------------------------------------- */
/* -- µGUI - Generic GUI module (C)Achim Döbler, 2014                            -- */
/* -------------------------------------------------------------------------------- */
// µGUI is a generic GUI module for embedded systems.
// This is a free software that is open for education, research and commercial
// developments under license policy of following terms.
//
//  Copyright (C) 2014, Achim Döbler, all rights reserved.
//  URL: http://www.embeddedlightning.com/
//
// * The µGUI module is a free software and there is NO WARRANTY.
// * No restriction on use. You can use, modify and redistribute it for
//   personal, non-profit or commercial products UNDER YOUR RESPONSIBILITY.
// * Redistributions of source code must retain the above copyright notice.
//
/* -------------------------------------------------------------------------------- */
/* -- REVISION HISTORY                                                           -- */
/* -------------------------------------------------------------------------------- */
//  Oct 11, 2014  V0.1  First release.
/* -------------------------------------------------------------------------------- */


#ifndef __UGUI_H
#define __UGUI_H

#include "stdint.h"//wn

/* -------------------------------------------------------------------------------- */
/* -- CONFIG SECTION                                                             -- */
/* -------------------------------------------------------------------------------- */

/* Enable needed fonts here */
//#define  USE_FONT_4X6
//#define  USE_FONT_5X8
//#define  USE_FONT_5X12
//#define  USE_FONT_6X8
//#define  USE_FONT_6X10
//#define  USE_FONT_7X12
//#define  USE_FONT_8X8
//#define  USE_FONT_8X12
//#define  USE_FONT_8X14
//#define  USE_FONT_10X16
//#define  USE_FONT_12X16
//#define  USE_FONT_12X20
//#define  USE_FONT_16X26
#define  USE_FONT_22X36
//#define  USE_FONT_24X40
//#define  USE_FONT_32X53

/* Specify platform-dependent integer types here */

typedef uint8_t      UG_U8;
typedef int8_t       UG_S8;
typedef uint16_t     UG_U16;
typedef int16_t      UG_S16;
typedef uint32_t     UG_U32;
typedef int32_t      UG_S32;


/* Example for dsPIC33
typedef unsigned char         UG_U8;
typedef signed char           UG_S8;
typedef unsigned int          UG_U16;
typedef signed int            UG_S16;
typedef unsigned long int     UG_U32;
typedef signed long int       UG_S32;
*/

/* -------------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------------- */



/* -------------------------------------------------------------------------------- */
/* -- INTERNAL THINGS                                                            -- */
/* -------------------------------------------------------------------------------- */

#ifndef NULL
   #define NULL ((void*) 0)
#endif

typedef UG_U32 UG_COLOR;

#define  FONT_4X6          0
#define  FONT_5X8          1
#define  FONT_5X12         2
#define  FONT_6X8          3
#define  FONT_6X10         4
#define  FONT_7X12         5
#define  FONT_8X8          6
#define  FONT_8X12         7
#define  FONT_8X14         8
#define  FONT_10X16        9
#define  FONT_12X16        10
#define  FONT_12X20        11
#define  FONT_16X26        12
#define  FONT_22X36        13
#define  FONT_24X40        14
#define  FONT_32X53        15

/* GUI structure */
typedef struct
{
   void (*pset)(UG_S16,UG_S16,UG_COLOR);
   UG_S16 x_dim;
   UG_S16 y_dim;
   struct
   {
      UG_S16 x_pos;
      UG_S16 y_pos;
      UG_S16 x_start;
      UG_S16 y_start;
      UG_S16 x_end;
      UG_S16 y_end;
      UG_COLOR fore_color;
      UG_COLOR back_color;
   } console;
   struct
   {
      unsigned char* p;
      UG_S16 char_width;
      UG_S16 char_height;
      UG_S16 char_h_space;
      UG_S16 char_v_space;
   } font;
   UG_COLOR fore_color;
   UG_COLOR back_color;
   UG_U16 status;
} UG_GUI;

typedef struct
{
   unsigned char* p;
   UG_S16 char_width;
   UG_S16 char_height;
} UG_FONT;


/* Some colors */
// Source: http://www.rapidtables.com/web/color/RGB_Color.htm
#define  C_MAROON                     0x800000
#define  C_DARK_RED                   0x8B0000
#define  C_BROWN                      0xA52A2A
#define  C_FIREBRICK                  0xB22222
#define  C_CRIMSON                    0xDC143C
#define  C_RED                        0xFF0000
#define  C_TOMATO                     0xFF6347
#define  C_CORAL                      0xFF7F50
#define  C_INDIAN_RED                 0xCD5C5C
#define  C_LIGHT_CORAL                0xF08080
#define  C_DARK_SALMON                0xE9967A
#define  C_SALMON                     0xFA8072
#define  C_LIGHT_SALMON               0xFFA07A
#define  C_ORANGE_RED                 0xFF4500
#define  C_DARK_ORANGE                0xFF8C00
#define  C_ORANGE                     0xFFA500
#define  C_GOLD                       0xFFD700
#define  C_DARK_GOLDEN_ROD            0xB8860B
#define  C_GOLDEN_ROD                 0xDAA520
#define  C_PALE_GOLDEN_ROD            0xEEE8AA
#define  C_DARK_KHAKI                 0xBDB76B
#define  C_KHAKI                      0xF0E68C
#define  C_OLIVE                      0x808000
#define  C_YELLOW                     0xFFFF00
#define  C_YELLOW_GREEN               0x9ACD32
#define  C_DARK_OLIVE_GREEN           0x556B2F
#define  C_OLIVE_DRAB                 0x6B8E23
#define  C_LAWN_GREEN                 0x7CFC00
#define  C_CHART_REUSE                0x7FFF00
#define  C_GREEN_YELLOW               0xADFF2F
#define  C_DARK_GREEN                 0x006400
#define  C_GREEN                      0x008000
#define  C_FOREST_GREEN               0x228B22
#define  C_LIME                       0x00FF00
#define  C_LIME_GREEN                 0x32CD32
#define  C_LIGHT_GREEN                0x90EE90
#define  C_PALE_GREEN                 0x98FB98
#define  C_DARK_SEA_GREEN             0x8FBC8F
#define  C_MEDIUM_SPRING_GREEN        0x00FA9A
#define  C_SPRING_GREEN               0x00FF7F
#define  C_SEA_GREEN                  0x2E8B57
#define  C_MEDIUM_AQUA_MARINE         0x66CDAA
#define  C_MEDIUM_SEA_GREEN           0x3CB371
#define  C_LIGHT_SEA_GREEN            0x20B2AA
#define  C_DARK_SLATE_GRAY            0x2F4F4F
#define  C_TEAL                       0x008080
#define  C_DARK_CYAN                  0x008B8B
#define  C_AQUA                       0x00FFFF
#define  C_CYAN                       0x00FFFF
#define  C_LIGHT_CYAN                 0xE0FFFF
#define  C_DARK_TURQUOISE             0x00CED1
#define  C_TURQUOISE                  0x40E0D0
#define  C_MEDIUM_TURQUOISE           0x48D1CC
#define  C_PALE_TURQUOISE             0xAFEEEE
#define  C_AQUA_MARINE                0x7FFFD4
#define  C_POWDER_BLUE                0xB0E0E6
#define  C_CADET_BLUE                 0x5F9EA0
#define  C_STEEL_BLUE                 0x4682B4
#define  C_CORN_FLOWER_BLUE           0x6495ED
#define  C_DEEP_SKY_BLUE              0x00BFFF
#define  C_DODGER_BLUE                0x1E90FF
#define  C_LIGHT_BLUE                 0xADD8E6
#define  C_SKY_BLUE                   0x87CEEB
#define  C_LIGHT_SKY_BLUE             0x87CEFA
#define  C_MIDNIGHT_BLUE              0x191970
#define  C_NAVY                       0x000080
#define  C_DARK_BLUE                  0x00008B
#define  C_MEDIUM_BLUE                0x0000CD
#define  C_BLUE                       0x0000FF
#define  C_ROYAL_BLUE                 0x4169E1
#define  C_BLUE_VIOLET                0x8A2BE2
#define  C_INDIGO                     0x4B0082
#define  C_DARK_SLATE_BLUE            0x483D8B
#define  C_SLATE_BLUE                 0x6A5ACD
#define  C_MEDIUM_SLATE_BLUE          0x7B68EE
#define  C_MEDIUM_PURPLE              0x9370DB
#define  C_DARK_MAGENTA               0x8B008B
#define  C_DARK_VIOLET                0x9400D3
#define  C_DARK_ORCHID                0x9932CC
#define  C_MEDIUM_ORCHID              0xBA55D3
#define  C_PURPLE                     0x800080
#define  C_THISTLE                    0xD8BFD8
#define  C_PLUM                       0xDDA0DD
#define  C_VIOLET                     0xEE82EE
#define  C_MAGENTA                    0xFF00FF
#define  C_ORCHID                     0xDA70D6
#define  C_MEDIUM_VIOLET_RED          0xC71585
#define  C_PALE_VIOLET_RED            0xDB7093
#define  C_DEEP_PINK                  0xFF1493
#define  C_HOT_PINK                   0xFF69B4
#define  C_LIGHT_PINK                 0xFFB6C1
#define  C_PINK                       0xFFC0CB
#define  C_ANTIQUE_WHITE              0xFAEBD7
#define  C_BEIGE                      0xF5F5DC
#define  C_BISQUE                     0xFFE4C4
#define  C_BLANCHED_ALMOND            0xFFEBCD
#define  C_WHEAT                      0xF5DEB3
#define  C_CORN_SILK                  0xFFF8DC
#define  C_LEMON_CHIFFON              0xFFFACD
#define  C_LIGHT_GOLDEN_ROD_YELLOW    0xFAFAD2
#define  C_LIGHT_YELLOW               0xFFFFE0
#define  C_SADDLE_BROWN               0x8B4513
#define  C_SIENNA                     0xA0522D
#define  C_CHOCOLATE                  0xD2691E
#define  C_PERU                       0xCD853F
#define  C_SANDY_BROWN                0xF4A460
#define  C_BURLY_WOOD                 0xDEB887
#define  C_TAN                        0xD2B48C
#define  C_ROSY_BROWN                 0xBC8F8F
#define  C_MOCCASIN                   0xFFE4B5
#define  C_NAVAJO_WHITE               0xFFDEAD
#define  C_PEACH_PUFF                 0xFFDAB9
#define  C_MISTY_ROSE                 0xFFE4E1
#define  C_LAVENDER_BLUSH             0xFFF0F5
#define  C_LINEN                      0xFAF0E6
#define  C_OLD_LACE                   0xFDF5E6
#define  C_PAPAYA_WHIP                0xFFEFD5
#define  C_SEA_SHELL                  0xFFF5EE
#define  C_MINT_CREAM                 0xF5FFFA
#define  C_SLATE_GRAY                 0x708090
#define  C_LIGHT_SLATE_GRAY           0x778899
#define  C_LIGHT_STEEL_BLUE           0xB0C4DE
#define  C_LAVENDER                   0xE6E6FA
#define  C_FLORAL_WHITE               0xFFFAF0
#define  C_ALICE_BLUE                 0xF0F8FF
#define  C_GHOST_WHITE                0xF8F8FF
#define  C_HONEYDEW                   0xF0FFF0
#define  C_IVORY                      0xFFFFF0
#define  C_AZURE                      0xF0FFFF
#define  C_SNOW                       0xFFFAFA
#define  C_BLACK                      0x000000
#define  C_DIM_GRAY                   0x696969
#define  C_GRAY                       0x808080
#define  C_DARK_GRAY                  0xA9A9A9
#define  C_SILVER                     0xC0C0C0
#define  C_LIGHT_GRAY                 0xD3D3D3
#define  C_GAINSBORO                  0xDCDCDC
#define  C_WHITE_SMOKE                0xF5F5F5
#define  C_WHITE                      0xFFFFFF


/* -------------------------------------------------------------------------------- */
/* -- PROTOTYPES                                                                 -- */
/* -------------------------------------------------------------------------------- */

UG_S16 UG_Init( UG_GUI* g, void (*p)(UG_S16,UG_S16,UG_COLOR), UG_S16 x, UG_S16 y );
UG_S16 UG_SelectGUI( UG_GUI* g );
void UG_FontSelect( UG_U16 font );
void UG_FillScreen( UG_COLOR c );
void UG_FillFrame( UG_S16 x1, UG_S16 y1, UG_S16 x2, UG_S16 y2, UG_COLOR c );
void UG_DrawMesh( UG_S16 x1, UG_S16 y1, UG_S16 x2, UG_S16 y2, UG_COLOR c );
void UG_DrawFrame( UG_S16 x1, UG_S16 y1, UG_S16 x2, UG_S16 y2, UG_COLOR c );
void UG_DrawPixel( UG_S16 x0, UG_S16 y0, UG_COLOR c );
void UG_DrawCircle( UG_S16 x0, UG_S16 y0, UG_S16 r, UG_COLOR c );
void UG_FillCircle( UG_S16 x0, UG_S16 y0, UG_S16 r, UG_COLOR c );
void UG_DrawLine( UG_S16 x1, UG_S16 y1, UG_S16 x2, UG_S16 y2, UG_COLOR c );
void UG_PutString( UG_S16 x, UG_S16 y, char* str );
void UG_PutChar( char chr, UG_S16 x, UG_S16 y, UG_COLOR fc, UG_COLOR bc );
void UG_ConsolePutString( char* str );
void UG_ConsoleSetArea( UG_S16 xs, UG_S16 ys, UG_S16 xe, UG_S16 ye );
void UG_ConsoleSetForecolor( UG_COLOR c );
void UG_ConsoleSetBackcolor( UG_COLOR c );
void UG_SetForecolor( UG_COLOR c );
void UG_SetBackcolor( UG_COLOR c );
UG_S16 UG_GetXDim( void );
UG_S16 UG_GetYDim( void );
void UG_FontSetHSpace( UG_U16 s );
void UG_FontSetVSpace( UG_U16 s );

#endif

