#pragma once
#include <Adafruit_GFX.h>

const uint8_t HD44780Bitmaps[] PROGMEM = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x21, 0x08, 0x40, 0x00, 0x80, 0x52, 0x94,
    0x00, 0x00, 0x00, 0x52, 0xbe, 0xaf, 0xa9, 0x40, 0x23, 0xe8, 0xe2, 0xf8,
    0x80, 0xc6, 0x44, 0x44, 0x4c, 0x60, 0x64, 0xa8, 0x8a, 0xc9, 0xa0, 0x61,
    0x10, 0x00, 0x00, 0x00, 0x11, 0x10, 0x84, 0x10, 0x40, 0x41, 0x04, 0x21,
    0x11, 0x00, 0x01, 0x2a, 0xea, 0x90, 0x00, 0x01, 0x09, 0xf2, 0x10, 0x00,
    0x00, 0x00, 0x06, 0x11, 0x00, 0x00, 0x01, 0xf0, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x31, 0x80, 0x00, 0x44, 0x44, 0x40, 0x00, 0x74, 0x67, 0x5c, 0xc5,
    0xc0, 0x23, 0x08, 0x42, 0x11, 0xc0, 0x74, 0x42, 0x22, 0x23, 0xe0, 0xf8,
    0x88, 0x20, 0xc5, 0xc0, 0x11, 0x95, 0x2f, 0x88, 0x40, 0xfc, 0x3c, 0x10,
    0xc5, 0xc0, 0x32, 0x21, 0xe8, 0xc5, 0xc0, 0xfc, 0x42, 0x22, 0x10, 0x80,
    0x74, 0x62, 0xe8, 0xc5, 0xc0, 0x74, 0x62, 0xf0, 0x89, 0x80, 0x03, 0x18,
    0x06, 0x30, 0x00, 0x03, 0x18, 0x06, 0x11, 0x00, 0x11, 0x11, 0x04, 0x10,
    0x40, 0x00, 0x3e, 0x0f, 0x80, 0x00, 0x41, 0x04, 0x11, 0x11, 0x00, 0x74,
    0x42, 0x22, 0x00, 0x80, 0x74, 0x42, 0xda, 0xd5, 0xc0, 0x74, 0x63, 0x1f,
    0xc6, 0x20, 0xf4, 0x63, 0xe8, 0xc7, 0xc0, 0x74, 0x61, 0x08, 0x45, 0xc0,
    0xe4, 0xa3, 0x18, 0xcb, 0x80, 0xfc, 0x21, 0xe8, 0x43, 0xe0, 0xfc, 0x21,
    0xe8, 0x42, 0x00, 0x74, 0x61, 0x78, 0xc5, 0xe0, 0x8c, 0x63, 0xf8, 0xc6,
    0x20, 0x71, 0x08, 0x42, 0x11, 0xc0, 0x38, 0x84, 0x21, 0x49, 0x80, 0x8c,
    0xa9, 0x8a, 0x4a, 0x20, 0x84, 0x21, 0x08, 0x43, 0xe0, 0x8e, 0xeb, 0x58,
    0xc6, 0x20, 0x8c, 0x73, 0x59, 0xc6, 0x20, 0x74, 0x63, 0x18, 0xc5, 0xc0,
    0xf4, 0x63, 0xe8, 0x42, 0x00, 0x74, 0x63, 0x1a, 0xc9, 0xa0, 0xf4, 0x63,
    0xea, 0x4a, 0x20, 0x7c, 0x20, 0xe0, 0x87, 0xc0, 0xf9, 0x08, 0x42, 0x10,
    0x80, 0x8c, 0x63, 0x18, 0xc5, 0xc0, 0x8c, 0x63, 0x18, 0xa8, 0x80, 0x8c,
    0x63, 0x5a, 0xd5, 0x40, 0x8c, 0x54, 0x45, 0x46, 0x20, 0x8c, 0x62, 0xa2,
    0x10, 0x80, 0xf8, 0x44, 0x44, 0x43, 0xe0, 0xe4, 0x21, 0x08, 0x43, 0x80,
    0x8a, 0xbe, 0x4f, 0x90, 0x80, 0x70, 0x84, 0x21, 0x09, 0xc0, 0x22, 0xa2,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xe0, 0x41, 0x04, 0x00, 0x00,
    0x00, 0x00, 0x1c, 0x17, 0xc5, 0xe0, 0x84, 0x2d, 0x98, 0xc7, 0xc0, 0x00,
    0x1d, 0x08, 0x45, 0xc0, 0x08, 0x5b, 0x38, 0xc5, 0xe0, 0x00, 0x1d, 0x1f,
    0xc1, 0xc0, 0x32, 0x51, 0xc4, 0x21, 0x00, 0x03, 0xe3, 0x17, 0x85, 0xc0,
    0x84, 0x2d, 0x98, 0xc6, 0x20, 0x20, 0x18, 0x42, 0x11, 0xc0, 0x10, 0x0c,
    0x21, 0x49, 0x80, 0x84, 0x25, 0x4c, 0x52, 0x40, 0x61, 0x08, 0x42, 0x11,
    0xc0, 0x00, 0x35, 0x5a, 0xc6, 0x20, 0x00, 0x2d, 0x98, 0xc6, 0x20, 0x00,
    0x1d, 0x18, 0xc5, 0xc0, 0x00, 0x3d, 0x1f, 0x42, 0x00, 0x00, 0x1b, 0x37,
    0x84, 0x20, 0x00, 0x2d, 0x98, 0x42, 0x00, 0x00, 0x1d, 0x07, 0x07, 0xc0,
    0x42, 0x38, 0x84, 0x24, 0xc0, 0x00, 0x23, 0x18, 0xcd, 0xa0, 0x00, 0x23,
    0x18, 0xa8, 0x80, 0x00, 0x23, 0x5a, 0xd5, 0x40, 0x00, 0x22, 0xa2, 0x2a,
    0x20, 0x00, 0x23, 0x17, 0x85, 0xc0, 0x00, 0x3e, 0x22, 0x23, 0xe0, 0x11,
    0x08, 0x82, 0x10, 0x40, 0x21, 0x08, 0x42, 0x10, 0x80, 0x41, 0x08, 0x22,
    0x11, 0x00, 0x01, 0x05, 0xf1, 0x10, 0x00, 0x01, 0x11, 0xf4, 0x10, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x21, 0x08, 0x40, 0x00, 0x80, 0x52, 0x94,
    0x00, 0x00, 0x00, 0x52, 0xbe, 0xaf, 0xa9, 0x40, 0x23, 0xe8, 0xe2, 0xf8,
    0x80, 0xc6, 0x44, 0x44, 0x4c, 0x60, 0x64, 0xa8, 0x8a, 0xc9, 0xa0, 0x61,
    0x10, 0x00, 0x00, 0x00, 0x11, 0x10, 0x84, 0x10, 0x40, 0x41, 0x04, 0x21,
    0x11, 0x00, 0x01, 0x2a, 0xea, 0x90, 0x00, 0x01, 0x09, 0xf2, 0x10, 0x00,
    0x00, 0x00, 0x06, 0x11, 0x00, 0x00, 0x01, 0xf0, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x31, 0x80, 0x00, 0x44, 0x44, 0x40, 0x00, 0x74, 0x67, 0x5c, 0xc5,
    0xc0, 0x23, 0x08, 0x42, 0x11, 0xc0, 0x74, 0x42, 0x22, 0x23, 0xe0, 0xf8,
    0x88, 0x20, 0xc5, 0xc0, 0x11, 0x95, 0x2f, 0x88, 0x40, 0xfc, 0x3c, 0x10,
    0xc5, 0xc0, 0x32, 0x21, 0xe8, 0xc5, 0xc0, 0xfc, 0x42, 0x22, 0x10, 0x80,
    0x74, 0x62, 0xe8, 0xc5, 0xc0, 0x74, 0x62, 0xf0, 0x89, 0x80, 0x03, 0x18,
    0x06, 0x30, 0x00, 0x03, 0x18, 0x06, 0x11, 0x00, 0x11, 0x11, 0x04, 0x10,
    0x40, 0x00, 0x3e, 0x0f, 0x80, 0x00, 0x41, 0x04, 0x11, 0x11, 0x00, 0x74,
    0x42, 0x22, 0x00, 0x80, 0x74, 0x42, 0xda, 0xd5, 0xc0, 0x74, 0x63, 0x1f,
    0xc6, 0x20, 0xf4, 0x63, 0xe8, 0xc7, 0xc0, 0x74, 0x61, 0x08, 0x45, 0xc0,
    0xe4, 0xa3, 0x18, 0xcb, 0x80, 0xfc, 0x21, 0xe8, 0x43, 0xe0, 0xfc, 0x21,
    0xe8, 0x42, 0x00, 0x74, 0x61, 0x78, 0xc5, 0xe0, 0x8c, 0x63, 0xf8, 0xc6,
    0x20, 0x71, 0x08, 0x42, 0x11, 0xc0, 0x38, 0x84, 0x21, 0x49, 0x80, 0x8c,
    0xa9, 0x8a, 0x4a, 0x20, 0x84, 0x21, 0x08, 0x43, 0xe0, 0x8e, 0xeb, 0x58,
    0xc6, 0x20, 0x8c, 0x73, 0x59, 0xc6, 0x20, 0x74, 0x63, 0x18, 0xc5, 0xc0,
    0xf4, 0x63, 0xe8, 0x42, 0x00, 0x74, 0x63, 0x1a, 0xc9, 0xa0, 0xf4, 0x63,
    0xea, 0x4a, 0x20, 0x7c, 0x20, 0xe0, 0x87, 0xc0, 0xf9, 0x08, 0x42, 0x10,
    0x80, 0x8c, 0x63, 0x18, 0xc5, 0xc0, 0x8c, 0x63, 0x18, 0xa8, 0x80, 0x8c,
    0x63, 0x5a, 0xd5, 0x40, 0x8c, 0x54, 0x45, 0x46, 0x20, 0x8c, 0x62, 0xa2,
    0x10, 0x80, 0xf8, 0x44, 0x44, 0x43, 0xe0, 0xe4, 0x21, 0x08, 0x43, 0x80,
    0x8a, 0xbe, 0x4f, 0x90, 0x80, 0x70, 0x84, 0x21, 0x09, 0xc0, 0x22, 0xa2,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xe0, 0x41, 0x04, 0x00, 0x00,
    0x00, 0x00, 0x1c, 0x17, 0xc5, 0xe0, 0x84, 0x2d, 0x98, 0xc7, 0xc0, 0x00,
    0x1d, 0x08, 0x45, 0xc0, 0x08, 0x5b, 0x38, 0xc5, 0xe0, 0x00, 0x1d, 0x1f,
    0xc1, 0xc0, 0x32, 0x51, 0xc4, 0x21, 0x00, 0x03, 0xe3, 0x17, 0x85, 0xc0,
    0x84, 0x2d, 0x98, 0xc6, 0x20, 0x20, 0x18, 0x42, 0x11, 0xc0, 0x10, 0x0c,
    0x21, 0x49, 0x80, 0x84, 0x25, 0x4c, 0x52, 0x40, 0x61, 0x08, 0x42, 0x11,
    0xc0, 0x00, 0x35, 0x5a, 0xc6, 0x20, 0x00, 0x2d, 0x98, 0xc6, 0x20, 0x00,
    0x1d, 0x18, 0xc5, 0xc0, 0x00, 0x3d, 0x1f, 0x42, 0x00, 0x00, 0x1b, 0x37,
    0x84, 0x20, 0x00, 0x2d, 0x98, 0x42, 0x00, 0x00, 0x1d, 0x07, 0x07, 0xc0,
    0x42, 0x38, 0x84, 0x24, 0xc0, 0x00, 0x23, 0x18, 0xcd, 0xa0, 0x00, 0x23,
    0x18, 0xa8, 0x80, 0x00, 0x23, 0x5a, 0xd5, 0x40, 0x00, 0x22, 0xa2, 0x2a,
    0x20, 0x00, 0x23, 0x17, 0x85, 0xc0, 0x00, 0x3e, 0x22, 0x23, 0xe0, 0x11,
    0x08, 0x82, 0x10, 0x40, 0x21, 0x08, 0x42, 0x10, 0x80, 0x41, 0x08, 0x22,
    0x11, 0x00, 0x01, 0x05, 0xf1, 0x10, 0x00, 0x01, 0x11, 0xf4, 0x10, 0x00};
const GFXglyph HD44780Glyphs[] PROGMEM = {
  {0, 5, 7, 6, 0, -6},
  {5, 5, 7, 6, 0, -6},
  {10, 5, 7, 6, 0, -6},
  {15, 5, 7, 6, 0, -6},
  {20, 5, 7, 6, 0, -6},
  {25, 5, 7, 6, 0, -6},
  {30, 5, 7, 6, 0, -6},
  {35, 5, 7, 6, 0, -6},
  {40, 5, 7, 6, 0, -6},
  {45, 5, 7, 6, 0, -6},
  {50, 5, 7, 6, 0, -6},
  {55, 5, 7, 6, 0, -6},
  {60, 5, 7, 6, 0, -6},
  {65, 5, 7, 6, 0, -6},
  {70, 5, 7, 6, 0, -6},
  {75, 5, 7, 6, 0, -6},
  {80, 5, 7, 6, 0, -6},
  {85, 5, 7, 6, 0, -6},
  {90, 5, 7, 6, 0, -6},
  {95, 5, 7, 6, 0, -6},
  {100, 5, 7, 6, 0, -6},
  {105, 5, 7, 6, 0, -6},
  {110, 5, 7, 6, 0, -6},
  {115, 5, 7, 6, 0, -6},
  {120, 5, 7, 6, 0, -6},
  {125, 5, 7, 6, 0, -6},
  {130, 5, 7, 6, 0, -6},
  {135, 5, 7, 6, 0, -6},
  {140, 5, 7, 6, 0, -6},
  {145, 5, 7, 6, 0, -6},
  {150, 5, 7, 6, 0, -6},
  {155, 5, 7, 6, 0, -6},
  {160, 5, 7, 6, 0, -6},
  {165, 5, 7, 6, 0, -6},
  {170, 5, 7, 6, 0, -6},
  {175, 5, 7, 6, 0, -6},
  {180, 5, 7, 6, 0, -6},
  {185, 5, 7, 6, 0, -6},
  {190, 5, 7, 6, 0, -6},
  {195, 5, 7, 6, 0, -6},
  {200, 5, 7, 6, 0, -6},
  {205, 5, 7, 6, 0, -6},
  {210, 5, 7, 6, 0, -6},
  {215, 5, 7, 6, 0, -6},
  {220, 5, 7, 6, 0, -6},
  {225, 5, 7, 6, 0, -6},
  {230, 5, 7, 6, 0, -6},
  {235, 5, 7, 6, 0, -6},
  {240, 5, 7, 6, 0, -6},
  {245, 5, 7, 6, 0, -6},
  {250, 5, 7, 6, 0, -6},
  {255, 5, 7, 6, 0, -6},
  {260, 5, 7, 6, 0, -6},
  {265, 5, 7, 6, 0, -6},
  {270, 5, 7, 6, 0, -6},
  {275, 5, 7, 6, 0, -6},
  {280, 5, 7, 6, 0, -6},
  {285, 5, 7, 6, 0, -6},
  {290, 5, 7, 6, 0, -6},
  {295, 5, 7, 6, 0, -6},
  {300, 5, 7, 6, 0, -6},
  {305, 5, 7, 6, 0, -6},
  {310, 5, 7, 6, 0, -6},
  {315, 5, 7, 6, 0, -6},
  {320, 5, 7, 6, 0, -6},
  {325, 5, 7, 6, 0, -6},
  {330, 5, 7, 6, 0, -6},
  {335, 5, 7, 6, 0, -6},
  {340, 5, 7, 6, 0, -6},
  {345, 5, 7, 6, 0, -6},
  {350, 5, 7, 6, 0, -6},
  {355, 5, 7, 6, 0, -6},
  {360, 5, 7, 6, 0, -6},
  {365, 5, 7, 6, 0, -6},
  {370, 5, 7, 6, 0, -6},
  {375, 5, 7, 6, 0, -6},
  {380, 5, 7, 6, 0, -6},
  {385, 5, 7, 6, 0, -6},
  {390, 5, 7, 6, 0, -6},
  {395, 5, 7, 6, 0, -6},
  {400, 5, 7, 6, 0, -6},
  {405, 5, 7, 6, 0, -6},
  {410, 5, 7, 6, 0, -6},
  {415, 5, 7, 6, 0, -6},
  {420, 5, 7, 6, 0, -6},
  {425, 5, 7, 6, 0, -6},
  {430, 5, 7, 6, 0, -6},
  {435, 5, 7, 6, 0, -6},
  {440, 5, 7, 6, 0, -6},
  {445, 5, 7, 6, 0, -6},
  {450, 5, 7, 6, 0, -6},
  {455, 5, 7, 6, 0, -6},
  {460, 5, 7, 6, 0, -6},
  {465, 5, 7, 6, 0, -6},
  {470, 5, 7, 6, 0, -6},
  {475, 5, 7, 6, 0, -6},
  {480, 5, 7, 6, 0, -6},
  {485, 5, 7, 6, 0, -6},
  {490, 5, 7, 6, 0, -6},
  {495, 5, 7, 6, 0, -6},
  {500, 5, 7, 6, 0, -6},
  {505, 5, 7, 6, 0, -6},
  {510, 5, 7, 6, 0, -6},
  {515, 5, 7, 6, 0, -6},
  {520, 5, 7, 6, 0, -6},
  {525, 5, 7, 6, 0, -6},
  {530, 5, 7, 6, 0, -6},
  {535, 5, 7, 6, 0, -6},
  {540, 5, 7, 6, 0, -6},
  {545, 5, 7, 6, 0, -6},
  {550, 5, 7, 6, 0, -6},
  {555, 5, 7, 6, 0, -6},
  {560, 5, 7, 6, 0, -6},
  {565, 5, 7, 6, 0, -6},
  {570, 5, 7, 6, 0, -6},
  {575, 5, 7, 6, 0, -6},
  {580, 5, 7, 6, 0, -6},
  {585, 5, 7, 6, 0, -6},
  {590, 5, 7, 6, 0, -6},
  {595, 5, 7, 6, 0, -6},
  {600, 5, 7, 6, 0, -6},
  {605, 5, 7, 6, 0, -6},
  {610, 5, 7, 6, 0, -6},
  {615, 5, 7, 6, 0, -6},
  {620, 5, 7, 6, 0, -6},
  {625, 5, 7, 6, 0, -6},
  {630, 5, 7, 6, 0, -6},
  {635, 5, 7, 6, 0, -6},
  {640, 5, 7, 6, 0, -6},
  {645, 5, 7, 6, 0, -6},
  {650, 5, 7, 6, 0, -6},
  {655, 5, 7, 6, 0, -6},
  {660, 5, 7, 6, 0, -6},
  {665, 5, 7, 6, 0, -6},
  {670, 5, 7, 6, 0, -6},
  {675, 5, 7, 6, 0, -6},
  {680, 5, 7, 6, 0, -6},
  {685, 5, 7, 6, 0, -6},
  {690, 5, 7, 6, 0, -6},
  {695, 5, 7, 6, 0, -6},
  {700, 5, 7, 6, 0, -6},
  {705, 5, 7, 6, 0, -6},
  {710, 5, 7, 6, 0, -6},
  {715, 5, 7, 6, 0, -6},
  {720, 5, 7, 6, 0, -6},
  {725, 5, 7, 6, 0, -6},
  {730, 5, 7, 6, 0, -6},
  {735, 5, 7, 6, 0, -6},
  {740, 5, 7, 6, 0, -6},
  {745, 5, 7, 6, 0, -6},
  {750, 5, 7, 6, 0, -6},
  {755, 5, 7, 6, 0, -6},
  {760, 5, 7, 6, 0, -6},
  {765, 5, 7, 6, 0, -6},
  {770, 5, 7, 6, 0, -6},
  {775, 5, 7, 6, 0, -6},
  {780, 5, 7, 6, 0, -6},
  {785, 5, 7, 6, 0, -6},
  {790, 5, 7, 6, 0, -6},
  {795, 5, 7, 6, 0, -6},
  {800, 5, 7, 6, 0, -6},
  {805, 5, 7, 6, 0, -6},
  {810, 5, 7, 6, 0, -6},
  {815, 5, 7, 6, 0, -6},
  {820, 5, 7, 6, 0, -6},
  {825, 5, 7, 6, 0, -6},
  {830, 5, 7, 6, 0, -6},
  {835, 5, 7, 6, 0, -6},
  {840, 5, 7, 6, 0, -6},
  {845, 5, 7, 6, 0, -6},
  {850, 5, 7, 6, 0, -6},
  {855, 5, 7, 6, 0, -6},
  {860, 5, 7, 6, 0, -6},
  {865, 5, 7, 6, 0, -6},
  {870, 5, 7, 6, 0, -6},
  {875, 5, 7, 6, 0, -6},
  {880, 5, 7, 6, 0, -6},
  {885, 5, 7, 6, 0, -6},
  {890, 5, 7, 6, 0, -6},
  {895, 5, 7, 6, 0, -6},
  {900, 5, 7, 6, 0, -6},
  {905, 5, 7, 6, 0, -6},
  {910, 5, 7, 6, 0, -6},
  {915, 5, 7, 6, 0, -6},
  {920, 5, 7, 6, 0, -6},
  {925, 5, 7, 6, 0, -6},
  {930, 5, 7, 6, 0, -6},
  {935, 5, 7, 6, 0, -6},
  {940, 5, 7, 6, 0, -6},
  {945, 5, 7, 6, 0, -6},
  {950, 5, 7, 6, 0, -6},
  {955, 5, 7, 6, 0, -6},
};
const GFXfont HD44780 PROGMEM = {(uint8_t *)HD44780Bitmaps,
                      (GFXglyph *)HD44780Glyphs, 0x20,
                      0xe0, 8};