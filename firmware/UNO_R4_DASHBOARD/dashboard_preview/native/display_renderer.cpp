// Host display surface for the production C++ drawing code. No MCU/USB I/O.
// Only the ST7796 address-window and RGB565 memory-write operations are consumed.
#include "SPI.h"
#include "dashboard_ui.h"
#include "st7796_lcd.h"
#include <array>

namespace {
std::array<uint16_t, St7796Lcd::WIDTH * St7796Lcd::HEIGHT> pixels{};
bool data_mode=true;
uint8_t command=0, position=0, high_byte=0;
uint8_t address[4]{};
uint16_t x0=0,y0=0,x1=479,y1=319,x=0,y=0;
bool second_byte=false;
void copyRgb(uint8_t *rgb) {
  for(size_t i=0;i<pixels.size();++i) {
    const uint16_t color=pixels[i];
    rgb[i*3]=uint8_t(((color>>11)&31)*255/31);
    rgb[i*3+1]=uint8_t(((color>>5)&63)*255/63);
    rgb[i*3+2]=uint8_t((color&31)*255/31);
  }
}
}
DisplayPixelSink SPI;
void digitalWrite(uint8_t pin, uint8_t value) {
  if(pin==D9) data_mode=(value==HIGH);
}
uint8_t DisplayPixelSink::transfer(uint8_t value) {
  if(!data_mode) {
    command=value; position=0; second_byte=false;
    if(command==0x2C) {x=x0; y=y0;}
  } else if(command==0x2A || command==0x2B) {
    if(position<4) address[position++]=value;
    if(position==4) {
      const uint16_t first=(uint16_t(address[0])<<8)|address[1];
      const uint16_t last=(uint16_t(address[2])<<8)|address[3];
      if(command==0x2A) {x0=first;x1=last;} else {y0=first;y1=last;}
    }
  } else if(command==0x2C) {
    if(!second_byte) high_byte=value;
    else {
      if(x<St7796Lcd::WIDTH && y<St7796Lcd::HEIGHT)
        pixels[size_t(y)*St7796Lcd::WIDTH+x]=(uint16_t(high_byte)<<8)|value;
      if(++x>x1) {x=x0;if(++y>y1)y=y0;}
    }
    second_byte=!second_byte;
  }
  return 0;
}
void DisplayPixelSink::transfer(void *buffer,size_t length) {
  const auto *bytes=static_cast<const uint8_t*>(buffer);
  for(size_t i=0;i<length;++i) transfer(bytes[i]);
}
extern "C" void render_dashboard(const int32_t *v, int can_lost, uint8_t *rgb) {
  DashboardValues values{
    uint8_t(v[0]),uint8_t(v[1]),uint8_t(v[2]),uint8_t(v[3]),int16_t(v[4]),
    uint16_t(v[5]),int16_t(v[6]),int16_t(v[7]),int16_t(v[8]),int16_t(v[9]),uint32_t(v[10])};
  // Full redraw from the UNO-reported applied values, independent of previous clients.
  DashboardUi::begin(values,can_lost!=0);
  copyRgb(rgb);
}
extern "C" void render_swipe_test(int result, uint8_t *rgb) {
  DashboardUi::showSwipeTest(result == 1 ? 'L' : result == 2 ? 'R' : '-');
  copyRgb(rgb);
}
