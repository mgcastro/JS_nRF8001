#ifndef  _JS_NRF8001_H
#define _JS_NRF8001_H

#include <boards.h>
#include <lib_aci.h>
#include <aci_setup.h>
#include <SPI.h>

#include <avr/sleep.h>
#include <avr/interrupt.h>

#include "JS_services.h"

#define BOARD_REQN    10
#define BOARD_RDYN    3

void BLE_initialize();
void BLE_process();
int BLE_get();
unsigned char BLE_free();

#endif