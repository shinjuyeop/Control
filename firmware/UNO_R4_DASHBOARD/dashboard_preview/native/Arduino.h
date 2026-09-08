#pragma once
#include <cstddef>
#include <cstdint>
constexpr uint8_t D7=7, D8=8, D9=9, D10=10;
constexpr uint8_t HIGH=1, LOW=0, OUTPUT=1;
void digitalWrite(uint8_t pin, uint8_t value);
inline void pinMode(uint8_t, uint8_t) {}
inline void delay(unsigned long) {}
