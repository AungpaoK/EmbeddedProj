#ifndef USER_INTERFACE_H
#define USER_INTERFACE_H

#include <Arduino.h>

void uiBegin();
char uiReadKey();
void uiPrintLine(uint8_t row, const char text[]);

#endif
