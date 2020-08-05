/*********************************************************************
*                SEGGER Microcontroller GmbH & Co. KG                *
*        Solutions for real time microcontroller applications        *
*                           www.segger.com                           *
**********************************************************************
*                                                                    *
* C-file generated by                                                *
*                                                                    *
*        Bitmap Converter (ST) for emWin V5.44.                      *
*        Compiled Nov 10 2017, 08:52:20                              *
*                                                                    *
*        (c) 1998 - 2017 Segger Microcontroller GmbH & Co. KG        *
*                                                                    *
**********************************************************************
*                                                                    *
* Source file: clock_icon                                            *
* Dimensions:  83 * 77                                               *
* NumColors:   2                                                     *
*                                                                    *
**********************************************************************
*/

#include <stdlib.h>

#include "GUI.h"

#ifndef GUI_CONST_STORAGE
  #define GUI_CONST_STORAGE const
#endif

extern GUI_CONST_STORAGE GUI_BITMAP bmclock_icon;

/*********************************************************************
*
*       Palette
*
*  Description
*    The following are the entries of the palette table.
*    The entries are stored as a 32-bit values of which 24 bits are
*    actually used according to the following bit mask: 0xBBGGRR
*
*    The lower   8 bits represent the Red   component.
*    The middle  8 bits represent the Green component.
*    The highest 8 bits represent the Blue  component.
*/
static GUI_CONST_STORAGE GUI_COLOR _Colorsclock_icon[] = {
#if (GUI_USE_ARGB == 0)
  0x000000, 0xFFFFFF
#else
  0xFF000000, 0xFFFFFFFF
#endif

};

static GUI_CONST_STORAGE GUI_LOGPALETTE _Palclock_icon = {
  2,  // Number of entries
  0,  // No transparency
  &_Colorsclock_icon[0]
};

static GUI_CONST_STORAGE unsigned char _acclock_icon[] = {
  /* RLE: 1366 Pixels @ 000,000 */ 254, 0x00, 254, 0x00, 254, 0x00, 254, 0x00, 254, 0x00, 96, 0x00, 
  /* RLE: 009 Pixels @ 038,016 */ 9, 0x01, 
  /* RLE: 005 Pixels @ 047,016 */ 5, 0x00, 
  /* ABS: 002 Pixels @ 052,016 */ 0, 2, 0x01, 0x01, 
  /* RLE: 064 Pixels @ 054,016 */ 64, 0x00, 
  /* RLE: 019 Pixels @ 035,017 */ 19, 0x01, 
  /* RLE: 062 Pixels @ 054,017 */ 62, 0x00, 
  /* RLE: 022 Pixels @ 033,018 */ 22, 0x01, 
  /* RLE: 059 Pixels @ 055,018 */ 59, 0x00, 
  /* RLE: 024 Pixels @ 031,019 */ 24, 0x01, 
  /* RLE: 057 Pixels @ 055,019 */ 57, 0x00, 
  /* RLE: 026 Pixels @ 029,020 */ 26, 0x01, 
  /* RLE: 056 Pixels @ 055,020 */ 56, 0x00, 
  /* RLE: 009 Pixels @ 028,021 */ 9, 0x01, 
  /* RLE: 010 Pixels @ 037,021 */ 10, 0x00, 
  /* RLE: 009 Pixels @ 047,021 */ 9, 0x01, 
  /* RLE: 054 Pixels @ 056,021 */ 54, 0x00, 
  /* RLE: 008 Pixels @ 027,022 */ 8, 0x01, 
  /* RLE: 013 Pixels @ 035,022 */ 13, 0x00, 
  /* RLE: 008 Pixels @ 048,022 */ 8, 0x01, 
  /* RLE: 053 Pixels @ 056,022 */ 53, 0x00, 
  /* RLE: 007 Pixels @ 026,023 */ 7, 0x01, 
  /* RLE: 015 Pixels @ 033,023 */ 15, 0x00, 
  /* RLE: 008 Pixels @ 048,023 */ 8, 0x01, 
  /* RLE: 052 Pixels @ 056,023 */ 52, 0x00, 
  /* RLE: 007 Pixels @ 025,024 */ 7, 0x01, 
  /* RLE: 009 Pixels @ 032,024 */ 9, 0x00, 
  /* ABS: 002 Pixels @ 041,024 */ 0, 2, 0x01, 0x01, 
  /* RLE: 005 Pixels @ 043,024 */ 5, 0x00, 
  /* RLE: 004 Pixels @ 048,024 */ 4, 0x01, 
  /* RLE: 055 Pixels @ 052,024 */ 55, 0x00, 
  /* RLE: 007 Pixels @ 024,025 */ 7, 0x01, 
  /* RLE: 009 Pixels @ 031,025 */ 9, 0x00, 
  /* RLE: 004 Pixels @ 040,025 */ 4, 0x01, 
  /* RLE: 063 Pixels @ 044,025 */ 63, 0x00, 
  /* RLE: 005 Pixels @ 024,026 */ 5, 0x01, 
  /* RLE: 011 Pixels @ 029,026 */ 11, 0x00, 
  /* RLE: 004 Pixels @ 040,026 */ 4, 0x01, 
  /* RLE: 062 Pixels @ 044,026 */ 62, 0x00, 
  /* RLE: 006 Pixels @ 023,027 */ 6, 0x01, 
  /* RLE: 011 Pixels @ 029,027 */ 11, 0x00, 
  /* RLE: 004 Pixels @ 040,027 */ 4, 0x01, 
  /* RLE: 062 Pixels @ 044,027 */ 62, 0x00, 
  /* RLE: 005 Pixels @ 023,028 */ 5, 0x01, 
  /* RLE: 012 Pixels @ 028,028 */ 12, 0x00, 
  /* RLE: 004 Pixels @ 040,028 */ 4, 0x01, 
  /* RLE: 061 Pixels @ 044,028 */ 61, 0x00, 
  /* RLE: 005 Pixels @ 022,029 */ 5, 0x01, 
  /* RLE: 013 Pixels @ 027,029 */ 13, 0x00, 
  /* RLE: 004 Pixels @ 040,029 */ 4, 0x01, 
  /* RLE: 017 Pixels @ 044,029 */ 17, 0x00, 
  /* ABS: 002 Pixels @ 061,029 */ 0, 2, 0x01, 0x01, 
  /* RLE: 042 Pixels @ 063,029 */ 42, 0x00, 
  /* RLE: 005 Pixels @ 022,030 */ 5, 0x01, 
  /* RLE: 013 Pixels @ 027,030 */ 13, 0x00, 
  /* RLE: 004 Pixels @ 040,030 */ 4, 0x01, 
  /* RLE: 016 Pixels @ 044,030 */ 16, 0x00, 
  /* RLE: 004 Pixels @ 060,030 */ 4, 0x01, 
  /* RLE: 040 Pixels @ 064,030 */ 40, 0x00, 
  /* RLE: 005 Pixels @ 021,031 */ 5, 0x01, 
  /* RLE: 014 Pixels @ 026,031 */ 14, 0x00, 
  /* RLE: 004 Pixels @ 040,031 */ 4, 0x01, 
  /* RLE: 015 Pixels @ 044,031 */ 15, 0x00, 
  /* RLE: 005 Pixels @ 059,031 */ 5, 0x01, 
  /* RLE: 040 Pixels @ 064,031 */ 40, 0x00, 
  /* RLE: 005 Pixels @ 021,032 */ 5, 0x01, 
  /* RLE: 014 Pixels @ 026,032 */ 14, 0x00, 
  /* RLE: 004 Pixels @ 040,032 */ 4, 0x01, 
  /* RLE: 015 Pixels @ 044,032 */ 15, 0x00, 
  /* RLE: 006 Pixels @ 059,032 */ 6, 0x01, 
  /* RLE: 039 Pixels @ 065,032 */ 39, 0x00, 
  /* RLE: 004 Pixels @ 021,033 */ 4, 0x01, 
  /* RLE: 015 Pixels @ 025,033 */ 15, 0x00, 
  /* RLE: 004 Pixels @ 040,033 */ 4, 0x01, 
  /* RLE: 016 Pixels @ 044,033 */ 16, 0x00, 
  /* RLE: 005 Pixels @ 060,033 */ 5, 0x01, 
  /* RLE: 038 Pixels @ 065,033 */ 38, 0x00, 
  /* RLE: 005 Pixels @ 020,034 */ 5, 0x01, 
  /* RLE: 015 Pixels @ 025,034 */ 15, 0x00, 
  /* RLE: 004 Pixels @ 040,034 */ 4, 0x01, 
  /* RLE: 016 Pixels @ 044,034 */ 16, 0x00, 
  /* RLE: 005 Pixels @ 060,034 */ 5, 0x01, 
  /* RLE: 038 Pixels @ 065,034 */ 38, 0x00, 
  /* RLE: 005 Pixels @ 020,035 */ 5, 0x01, 
  /* RLE: 015 Pixels @ 025,035 */ 15, 0x00, 
  /* RLE: 004 Pixels @ 040,035 */ 4, 0x01, 
  /* RLE: 016 Pixels @ 044,035 */ 16, 0x00, 
  /* RLE: 005 Pixels @ 060,035 */ 5, 0x01, 
  /* RLE: 038 Pixels @ 065,035 */ 38, 0x00, 
  /* RLE: 005 Pixels @ 020,036 */ 5, 0x01, 
  /* RLE: 015 Pixels @ 025,036 */ 15, 0x00, 
  /* RLE: 004 Pixels @ 040,036 */ 4, 0x01, 
  /* RLE: 017 Pixels @ 044,036 */ 17, 0x00, 
  /* RLE: 004 Pixels @ 061,036 */ 4, 0x01, 
  /* RLE: 038 Pixels @ 065,036 */ 38, 0x00, 
  /* RLE: 005 Pixels @ 020,037 */ 5, 0x01, 
  /* RLE: 015 Pixels @ 025,037 */ 15, 0x00, 
  /* RLE: 004 Pixels @ 040,037 */ 4, 0x01, 
  /* RLE: 016 Pixels @ 044,037 */ 16, 0x00, 
  /* RLE: 005 Pixels @ 060,037 */ 5, 0x01, 
  /* RLE: 038 Pixels @ 065,037 */ 38, 0x00, 
  /* RLE: 005 Pixels @ 020,038 */ 5, 0x01, 
  /* RLE: 015 Pixels @ 025,038 */ 15, 0x00, 
  /* RLE: 004 Pixels @ 040,038 */ 4, 0x01, 
  /* RLE: 017 Pixels @ 044,038 */ 17, 0x00, 
  /* RLE: 004 Pixels @ 061,038 */ 4, 0x01, 
  /* RLE: 038 Pixels @ 065,038 */ 38, 0x00, 
  /* RLE: 005 Pixels @ 020,039 */ 5, 0x01, 
  /* RLE: 015 Pixels @ 025,039 */ 15, 0x00, 
  /* RLE: 006 Pixels @ 040,039 */ 6, 0x01, 
  /* RLE: 015 Pixels @ 046,039 */ 15, 0x00, 
  /* RLE: 004 Pixels @ 061,039 */ 4, 0x01, 
  /* RLE: 038 Pixels @ 065,039 */ 38, 0x00, 
  /* RLE: 005 Pixels @ 020,040 */ 5, 0x01, 
  /* RLE: 015 Pixels @ 025,040 */ 15, 0x00, 
  /* RLE: 008 Pixels @ 040,040 */ 8, 0x01, 
  /* RLE: 012 Pixels @ 048,040 */ 12, 0x00, 
  /* RLE: 005 Pixels @ 060,040 */ 5, 0x01, 
  /* RLE: 038 Pixels @ 065,040 */ 38, 0x00, 
  /* RLE: 005 Pixels @ 020,041 */ 5, 0x01, 
  /* RLE: 017 Pixels @ 025,041 */ 17, 0x00, 
  /* RLE: 008 Pixels @ 042,041 */ 8, 0x01, 
  /* RLE: 010 Pixels @ 050,041 */ 10, 0x00, 
  /* RLE: 005 Pixels @ 060,041 */ 5, 0x01, 
  /* RLE: 038 Pixels @ 065,041 */ 38, 0x00, 
  /* RLE: 005 Pixels @ 020,042 */ 5, 0x01, 
  /* RLE: 018 Pixels @ 025,042 */ 18, 0x00, 
  /* RLE: 009 Pixels @ 043,042 */ 9, 0x01, 
  /* RLE: 008 Pixels @ 052,042 */ 8, 0x00, 
  /* RLE: 005 Pixels @ 060,042 */ 5, 0x01, 
  /* RLE: 039 Pixels @ 065,042 */ 39, 0x00, 
  /* RLE: 004 Pixels @ 021,043 */ 4, 0x01, 
  /* RLE: 021 Pixels @ 025,043 */ 21, 0x00, 
  /* RLE: 008 Pixels @ 046,043 */ 8, 0x01, 
  /* RLE: 006 Pixels @ 054,043 */ 6, 0x00, 
  /* RLE: 005 Pixels @ 060,043 */ 5, 0x01, 
  /* RLE: 039 Pixels @ 065,043 */ 39, 0x00, 
  /* RLE: 005 Pixels @ 021,044 */ 5, 0x01, 
  /* RLE: 021 Pixels @ 026,044 */ 21, 0x00, 
  /* RLE: 009 Pixels @ 047,044 */ 9, 0x01, 
  /* RLE: 004 Pixels @ 056,044 */ 4, 0x00, 
  /* RLE: 005 Pixels @ 060,044 */ 5, 0x01, 
  /* RLE: 039 Pixels @ 065,044 */ 39, 0x00, 
  /* RLE: 005 Pixels @ 021,045 */ 5, 0x01, 
  /* RLE: 023 Pixels @ 026,045 */ 23, 0x00, 
  /* RLE: 007 Pixels @ 049,045 */ 7, 0x01, 
  /* RLE: 003 Pixels @ 056,045 */ 3, 0x00, 
  /* RLE: 005 Pixels @ 059,045 */ 5, 0x01, 
  /* RLE: 040 Pixels @ 064,045 */ 40, 0x00, 
  /* RLE: 005 Pixels @ 021,046 */ 5, 0x01, 
  /* RLE: 025 Pixels @ 026,046 */ 25, 0x00, 
  /* RLE: 005 Pixels @ 051,046 */ 5, 0x01, 
  /* RLE: 003 Pixels @ 056,046 */ 3, 0x00, 
  /* RLE: 005 Pixels @ 059,046 */ 5, 0x01, 
  /* RLE: 041 Pixels @ 064,046 */ 41, 0x00, 
  /* RLE: 005 Pixels @ 022,047 */ 5, 0x01, 
  /* RLE: 026 Pixels @ 027,047 */ 26, 0x00, 
  /* ABS: 005 Pixels @ 053,047 */ 0, 5, 0x01, 0x01, 0x00, 0x00, 0x00, 
  /* RLE: 006 Pixels @ 058,047 */ 6, 0x01, 
  /* RLE: 041 Pixels @ 064,047 */ 41, 0x00, 
  /* RLE: 006 Pixels @ 022,048 */ 6, 0x01, 
  /* RLE: 030 Pixels @ 028,048 */ 30, 0x00, 
  /* RLE: 005 Pixels @ 058,048 */ 5, 0x01, 
  /* RLE: 043 Pixels @ 063,048 */ 43, 0x00, 
  /* RLE: 005 Pixels @ 023,049 */ 5, 0x01, 
  /* RLE: 029 Pixels @ 028,049 */ 29, 0x00, 
  /* RLE: 005 Pixels @ 057,049 */ 5, 0x01, 
  /* RLE: 045 Pixels @ 062,049 */ 45, 0x00, 
  /* RLE: 005 Pixels @ 024,050 */ 5, 0x01, 
  /* RLE: 027 Pixels @ 029,050 */ 27, 0x00, 
  /* RLE: 006 Pixels @ 056,050 */ 6, 0x01, 
  /* RLE: 045 Pixels @ 062,050 */ 45, 0x00, 
  /* RLE: 006 Pixels @ 024,051 */ 6, 0x01, 
  /* RLE: 025 Pixels @ 030,051 */ 25, 0x00, 
  /* RLE: 006 Pixels @ 055,051 */ 6, 0x01, 
  /* RLE: 047 Pixels @ 061,051 */ 47, 0x00, 
  /* RLE: 006 Pixels @ 025,052 */ 6, 0x01, 
  /* RLE: 023 Pixels @ 031,052 */ 23, 0x00, 
  /* RLE: 007 Pixels @ 054,052 */ 7, 0x01, 
  /* RLE: 048 Pixels @ 061,052 */ 48, 0x00, 
  /* RLE: 006 Pixels @ 026,053 */ 6, 0x01, 
  /* RLE: 021 Pixels @ 032,053 */ 21, 0x00, 
  /* RLE: 007 Pixels @ 053,053 */ 7, 0x01, 
  /* RLE: 050 Pixels @ 060,053 */ 50, 0x00, 
  /* RLE: 007 Pixels @ 027,054 */ 7, 0x01, 
  /* RLE: 017 Pixels @ 034,054 */ 17, 0x00, 
  /* RLE: 008 Pixels @ 051,054 */ 8, 0x01, 
  /* RLE: 051 Pixels @ 059,054 */ 51, 0x00, 
  /* RLE: 009 Pixels @ 027,055 */ 9, 0x01, 
  /* RLE: 013 Pixels @ 036,055 */ 13, 0x00, 
  /* RLE: 009 Pixels @ 049,055 */ 9, 0x01, 
  /* RLE: 054 Pixels @ 058,055 */ 54, 0x00, 
  /* RLE: 012 Pixels @ 029,056 */ 12, 0x01, 
  /* RLE: 004 Pixels @ 041,056 */ 4, 0x00, 
  /* RLE: 011 Pixels @ 045,056 */ 11, 0x01, 
  /* RLE: 057 Pixels @ 056,056 */ 57, 0x00, 
  /* RLE: 025 Pixels @ 030,057 */ 25, 0x01, 
  /* RLE: 060 Pixels @ 055,057 */ 60, 0x00, 
  /* RLE: 022 Pixels @ 032,058 */ 22, 0x01, 
  /* RLE: 063 Pixels @ 054,058 */ 63, 0x00, 
  /* RLE: 017 Pixels @ 034,059 */ 17, 0x01, 
  /* RLE: 068 Pixels @ 051,059 */ 68, 0x00, 
  /* RLE: 013 Pixels @ 036,060 */ 13, 0x01, 
  /* RLE: 1362 Pixels @ 049,060 */ 254, 0x00, 254, 0x00, 254, 0x00, 254, 0x00, 254, 0x00, 92, 0x00, 
  0
};  // 439 bytes for 6391 pixels

GUI_CONST_STORAGE GUI_BITMAP bmclock_icon = {
  83, // xSize
  77, // ySize
  83, // BytesPerLine
  GUI_COMPRESS_RLE8, // BitsPerPixel
  _acclock_icon,  // Pointer to picture data (indices)
  &_Palclock_icon,  // Pointer to palette
  GUI_DRAW_RLE8
};

/*************************** End of file ****************************/
