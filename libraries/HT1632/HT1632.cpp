#include "HT1632.h"

#define HT1632_NUMBERS_ONLY

#ifdef HT1632_NUMBERS_ONLY
#include "glcd_font_numbers.c"
#else
#include "glcdfont.c"
#endif
//#include "tomthumbfont.c"

#define swap(a, b) { uint16_t t = a; a = b; b = t; }

HT1632LEDMatrix::HT1632LEDMatrix(uint8_t data, uint8_t wr, uint8_t cs1) {
  matrices = (HT1632 *)malloc(sizeof(HT1632));

  matrices[0] = HT1632(data, wr, cs1);
  matrixNum  = 1;
  _width = 24 * matrixNum;
  _height = 16;
}

HT1632LEDMatrix::HT1632LEDMatrix(uint8_t data, uint8_t wr, 
				 uint8_t cs1, uint8_t cs2) {
  matrices = (HT1632 *)malloc(2 * sizeof(HT1632));

  matrices[0] = HT1632(data, wr, cs1);
  matrices[1] = HT1632(data, wr, cs2);
  matrixNum  = 2;
  _width = 24 * matrixNum;
  _height = 16;
}

HT1632LEDMatrix::HT1632LEDMatrix(uint8_t data, uint8_t wr, 
				 uint8_t cs1, uint8_t cs2, uint8_t cs3) {
  matrices = (HT1632 *)malloc(3 * sizeof(HT1632));

  matrices[0] = HT1632(data, wr, cs1);
  matrices[1] = HT1632(data, wr, cs2);
  matrices[2] = HT1632(data, wr, cs3);
  matrixNum  = 3;
  _width = 24 * matrixNum;
  _height = 16;
}

HT1632LEDMatrix::HT1632LEDMatrix(uint8_t data, uint8_t wr, 
				 uint8_t cs1, uint8_t cs2, 
				 uint8_t cs3, uint8_t cs4) {
  matrices = (HT1632 *)malloc(4 * sizeof(HT1632));

  matrices[0] = HT1632(data, wr, cs1);
  matrices[1] = HT1632(data, wr, cs2);
  matrices[2] = HT1632(data, wr, cs3);
  matrices[3] = HT1632(data, wr, cs4);
  matrixNum  = 4;
  _width = 24 * matrixNum;
  _height = 16;
}


void HT1632LEDMatrix::setPixel(uint8_t x, uint8_t y) {
  drawPixel(x, y, 1);
}
void HT1632LEDMatrix::clrPixel(uint8_t x, uint8_t y) {
  drawPixel(x, y, 0);
}

void HT1632LEDMatrix::drawPixel(uint8_t x, uint8_t y, uint8_t color) {
  if (y >= _height) return;
  if (x >= _width) return;

  uint8_t m;
  // figure out which matrix controller it is
  m = x / 24;
  x %= 24;

  uint16_t i;

#ifdef ADAFRUIT_DISPLAY
  if (x < 8)
    i = 7;          // so i = 7 - x&7
  else if (x < 16)
    i = 128 + 7;    // so i = 128 + (7 - x&7)
  else
    i = 256 + 7;    // so i = 256 + (7 - x&7)
  i -= (x & 7);

  if (y < 8)
    y *= 2;  // so              i +=     16 * (y & 7)
  else
    y = (y - 8) * 2 + 1;  // so i += 8 + 16 * (y & 7)
  i += y * 8;

#else // AliExpress display https://www.aliexpress.us/item/2251832766729843.html
  i = 16 * (x & 7) + (y & 7);
  if (x >= 16)
    i += 256;
  else if (x >= 8)
    i += 128; 
  if (y < 8)
    i += 8; 
  
#endif

  if (color) 
    matrices[m].setPixel(i);
  else
    matrices[m].clrPixel(i);
}


uint8_t HT1632LEDMatrix::width() {
  return _width;
}

uint8_t HT1632LEDMatrix::height() {
  return _height;
}

void HT1632LEDMatrix::begin(uint8_t type) {
  for (uint8_t i=0; i<matrixNum; i++) {
    matrices[i].begin(type);
  }
}

void HT1632LEDMatrix::clearScreen() {
  for (uint8_t i=0; i<matrixNum; i++) {
    matrices[i].clearScreen();
  }
}

void HT1632LEDMatrix::fillScreen() {
  for (uint8_t i=0; i<matrixNum; i++) {
    matrices[i].fillScreen();
  }
}

void HT1632LEDMatrix::setBrightness(uint8_t b) {
  for (uint8_t i=0; i<matrixNum; i++) {
    matrices[i].setBrightness(b);
  }
}

void HT1632LEDMatrix::blink(boolean b) {
  for (uint8_t i=0; i<matrixNum; i++) {
    matrices[i].blink(b);
  }
}

void HT1632LEDMatrix::writeScreen() {
  for (uint8_t i=0; i<matrixNum; i++) {
    matrices[i].writeScreen();
  }
}

#ifdef GRAPHICS
// bresenham's algorithm - thx wikpedia
void HT1632LEDMatrix::drawLine(int8_t x0, int8_t y0, int8_t x1, int8_t y1, 
		      uint8_t color) {
  uint16_t steep = abs(y1 - y0) > abs(x1 - x0);
  if (steep) {
    swap(x0, y0);
    swap(x1, y1);
  }

  if (x0 > x1) {
    swap(x0, x1);
    swap(y0, y1);
  }

  uint16_t dx, dy;
  dx = x1 - x0;
  dy = abs(y1 - y0);

  int16_t err = dx / 2;
  int16_t ystep;

  if (y0 < y1) {
    ystep = 1;
  } else {
    ystep = -1;}

  for (; x0<=x1; x0++) {
    if (steep) {
      drawPixel(y0, x0, color);
    } else {
      drawPixel(x0, y0, color);
    }
    err -= dy;
    if (err < 0) {
      y0 += ystep;
      err += dx;
    }
  }
}

// draw a rectangle
void HT1632LEDMatrix::drawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, 
		      uint8_t color) {
  drawLine(x, y, x+w-1, y, color);
  drawLine(x, y+h-1, x+w-1, y+h-1, color);

  drawLine(x, y, x, y+h-1, color);
  drawLine(x+w-1, y, x+w-1, y+h-1, color);
}

// fill a rectangle
void HT1632LEDMatrix::fillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, 
		      uint8_t color) {
  for (uint8_t i=x; i<x+w; i++) {
    for (uint8_t j=y; j<y+h; j++) {
      drawPixel(i, j, color);
    }
  }
}



// draw a circle outline
void HT1632LEDMatrix::drawCircle(uint8_t x0, uint8_t y0, uint8_t r, 
			uint8_t color) {
  int16_t f = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x = 0;
  int16_t y = r;

  drawPixel(x0, y0+r, color);
  drawPixel(x0, y0-r, color);
  drawPixel(x0+r, y0, color);
  drawPixel(x0-r, y0, color);

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;
  
    drawPixel(x0 + x, y0 + y, color);
    drawPixel(x0 - x, y0 + y, color);
    drawPixel(x0 + x, y0 - y, color);
    drawPixel(x0 - x, y0 - y, color);
    
    drawPixel(x0 + y, y0 + x, color);
    drawPixel(x0 - y, y0 + x, color);
    drawPixel(x0 + y, y0 - x, color);
    drawPixel(x0 - y, y0 - x, color);
    
  }
}


// fill a circle
void HT1632LEDMatrix::fillCircle(uint8_t x0, uint8_t y0, uint8_t r, uint8_t color) {
  int16_t f = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x = 0;
  int16_t y = r;

  drawLine(x0, y0-r, x0, y0+r+1, color);

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;
  
    drawLine(x0+x, y0-y, x0+x, y0+y+1, color);
    drawLine(x0-x, y0-y, x0-x, y0+y+1, color);
    drawLine(x0+y, y0-x, x0+y, y0+x+1, color);
    drawLine(x0-y, y0-x, x0-y, y0+x+1, color);
  }
}
#endif // GRAPHICS

void HT1632LEDMatrix::setCursor(uint8_t x, uint8_t y) {
  cursor_x = x; 
  cursor_y = y;
}

void HT1632LEDMatrix::setTextSize(uint8_t s) {
  textsize = s;
}

void HT1632LEDMatrix::setTextColor(uint8_t c) {
  textcolor = c;
}

#if ARDUINO >= 100
size_t HT1632LEDMatrix::write(uint8_t c) {
#else
void HT1632LEDMatrix::write(uint8_t c) {
#endif
  if (c == '\n') {
    cursor_y += textsize*FONT_HEIGHT;
    cursor_x = 0;
  } else if (c == '\r') {
    // skip em
  } else {
    drawChar(cursor_x, cursor_y, c, textcolor, textsize);
    cursor_x += textsize*FONT_WIDTH;
  }
#if ARDUINO >= 100
  return 1;
#endif
}


// draw a character using 5x7 font
/*
void HT1632LEDMatrix::drawChar(uint8_t x, uint8_t y, char c, 
			      uint16_t color, uint8_t size) {
  for (uint8_t i =0; i<5; i++ ) {
    uint8_t line = pgm_read_byte(font+(c*5)+i);
    for (uint8_t j = 0; j<8; j++) {
      if (line & 0x1) {
	if (size == 1) // default size
	  drawPixel(x+i, y+j, color);
	else {  // big size
	  fillRect(x+i*size, y+j*size, size, size, color);
	} 
      }
      line >>= 1;
    }
  }
}
 */

// draw a character using 3x6 font
void HT1632LEDMatrix::drawCharRows(uint8_t x, uint8_t y, char c, 
			      uint16_t color, uint8_t size, uint8_t startrow, uint8_t endrow) {
  if (endrow > FONT_BITS_PER_COL) endrow = FONT_BITS_PER_COL;
  for (uint8_t i =0; i<FONT_BYTES_PER_GLYPH; i++ ) {
    uint8_t line = pgm_read_byte(font+((c-FONT_BASE_CODE)*FONT_BYTES_PER_GLYPH)+i);
    for (uint8_t j = 0; j<endrow; j++) {
	  if (j >= startrow) {
        if (line & 0x1) {
          if (size == 1) // default size
            drawPixel(x+i, y+j, color);
          else {  // big size
#ifdef GRAPHICS
			  fillRect(x+i*size, y+j*size, size, size, color);
#endif // GRAPHICS
          }
        } 
      }
      line >>= 1;
    }
  }
}

void HT1632LEDMatrix::drawCharTx(uint8_t x, uint8_t y, char c, char d, 
			      uint16_t color, uint8_t size, uint8_t row) {
  drawCharRows(x, y-row, c, color, size, row, 99);
  drawCharRows(x, y-row+FONT_BITS_PER_COL, d, color, size, 0, row);
}

void HT1632LEDMatrix::drawChar(uint8_t x, uint8_t y, char c, 
			      uint16_t color, uint8_t size) {
   drawCharRows(x, y, c, color, size, 0, 99);
}

  
void HT1632LEDMatrix::drawBitmap(uint8_t x, uint8_t y, 
			const uint8_t *bitmap, uint8_t w, uint8_t h,
			uint8_t color) {
  for (uint8_t j=0; j<h; j++) {
    for (uint8_t i=0; i<w; i++ ) {
      if (pgm_read_byte(bitmap + i + (j/8)*w) & _BV(j%8)) {
	drawPixel(x+i, y+j, color);
      }
    }
  }
}

//////////////////////////////////////////////////////////////////////////


HT1632::HT1632(int8_t data, int8_t wr, int8_t cs, int8_t rd) {
  _data = data;
  _wr = wr;
  _cs = cs;
  _rd = rd;

  for (uint8_t i=0; i<48; i++) {
    ledmatrix[i] = 0;
  }
}

void HT1632::begin(uint8_t type) {
  pinMode(_cs, OUTPUT);
  digitalWrite(_cs, HIGH);
  pinMode(_wr, OUTPUT);
  digitalWrite(_wr, HIGH);
  pinMode(_data, OUTPUT);
  
  if (_rd >= 0) {
    pinMode(_rd, OUTPUT);
    digitalWrite(_rd, HIGH);
  }

  sendcommand(HT1632_SYS_EN);
  sendcommand(HT1632_LED_ON);
  sendcommand(HT1632_BLINK_OFF);
  sendcommand(HT1632_MASTER_MODE);
  sendcommand(HT1632_INT_RC);
  sendcommand(type);
  sendcommand(HT1632_PWM_CONTROL | 0xF);
  
  WIDTH = 24;
  HEIGHT = 16;
}

void HT1632::setBrightness(uint8_t pwm) {
  if (pwm > 15) pwm = 15;
  sendcommand(HT1632_PWM_CONTROL | pwm);
}

void HT1632::blink(boolean blinky) {
  if (blinky) 
    sendcommand(HT1632_BLINK_ON);
  else
    sendcommand(HT1632_BLINK_OFF);
}

// dpwe: flipped upside down to work-around an assembly bug...
void HT1632::setPixel(uint16_t i) {
  //ledmatrix[i/8] |= _BV(i%8); 
  ledmatrix[47 - i/8] |= _BV(7-(i%8)); 
}

void HT1632::clrPixel(uint16_t i) {
  //ledmatrix[i/8] &= ~_BV(i%8); 
  ledmatrix[47 - i/8] &= ~_BV(7-(i%8)); 
}

void HT1632::dumpScreen() {
#ifdef ENABLE_DUMP_SCREEN
  Serial.println("---------------------------------------");

  for (uint16_t i=0; i<(WIDTH*HEIGHT/8); i++) {
    Serial.print("0x");
    Serial.print(ledmatrix[i], HEX);
    Serial.print(" ");
    if (i % 3 == 2) Serial.println();
  }

  Serial.println("\n---------------------------------------");
#endif // ARDUINO_AVR_TRINKET5
}

void HT1632::writeScreen() {

  digitalWrite(_cs, LOW);

  writedata(HT1632_WRITE, 3);
  // send with address 0
  writedata(0, 7);

  for (uint16_t i=0; i<(WIDTH*HEIGHT/8); i+=2) {
    uint16_t d = ledmatrix[i];
    d <<= 8;
    d |= ledmatrix[i+1];

    writedata(d, 16);
  }
  digitalWrite(_cs, HIGH);
}


void HT1632::clearScreen() {
  for (uint8_t i=0; i<(WIDTH*HEIGHT/8); i++) {
    ledmatrix[i] = 0;
  }
  writeScreen();
}


void HT1632::writedata(uint16_t d, uint8_t bits) {
  pinMode(_data, OUTPUT);
  for (uint8_t i=bits; i > 0; i--) {
    digitalWrite(_wr, LOW);
   if (d & _BV(i-1)) {
     digitalWrite(_data, HIGH);
   } else {
     digitalWrite(_data, LOW);
   }
  digitalWrite(_wr, HIGH);
  }
  pinMode(_data, INPUT);
}




void HT1632::writeRAM(uint8_t addr, uint8_t data) {
  //Serial.print("Writing 0x"); Serial.print(data&0xF, HEX);
  //Serial.print(" to 0x"); Serial.println(addr & 0x7F, HEX);

  uint16_t d = HT1632_WRITE;
  d <<= 7;
  d |= addr & 0x7F;
  d <<= 4;
  d |= data & 0xF;
 
  digitalWrite(_cs, LOW);
  writedata(d, 14);
  digitalWrite(_cs, HIGH);
}


void HT1632::sendcommand(uint8_t cmd) {
  uint16_t data = 0;
  data = HT1632_COMMAND;
  data <<= 8;
  data |= cmd;
  data <<= 1;
  
  digitalWrite(_cs, LOW);
  writedata(data, 12);
  digitalWrite(_cs, HIGH);  
}


void HT1632::fillScreen() {
  for (uint8_t i=0; i<(WIDTH*HEIGHT/8); i++) {
    ledmatrix[i] = 0xFF;
  }
  writeScreen();
}
