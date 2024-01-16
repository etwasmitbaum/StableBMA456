#pragma once

/* Forked from bma.h by GuruSR (https://www.github.com/GuruSR/StableBMA) 
 * This fork is to improve Watchy functionality based on board version (via RTCType).
 * Version 1.0, February  6, 2022 - Initial changes for Watchy usage.
 * Version 1.1, February  8, 2022 - Fixed readTemperatureF to show F properly.
 * Version 1.2, July     19, 2022 - Fixed readTemperatureF to include errors.  License Update.
 *
 * MIT License
 *
 * Copyright (c) 2020 Lewis He
 * Copyright (c) 2022 for StableBMA GuruSR
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 * StableBMA is a fork of:
 * bma.h- Arduino library for Bosch BMA423 accelerometer software library.
 * Created by Lewis He on July 27, 2020.
 * github:https://github.com/lewisxhe/BMA423_Library
*/

#ifdef  ARDUINO
#include <Arduino.h>
#else
#include <stdlib.h>
#endif

#include "bma456w.h"
#include <Wire.h>

#ifndef WATCHY_H
enum {
    DIRECTION_TOP_EDGE_UP     = 0,
    DIRECTION_BOTTOM_EDGE_UP  = 1,
    DIRECTION_LEFT_EDGE_UP    = 2,
    DIRECTION_RIGHT_EDGE_UP   = 3,
    DIRECTION_DISP_UP         = 4,
    DIRECTION_DISP_DOWN       = 5
} ;
#endif

// BMA For Tilt/DTap
#define BMA423x_INT1_PIN 14
#define BMA423x_INT2_PIN 12
#define BMA423x_INT1_MASK (1<<BMA423x_INT1_PIN)
#define BMA423x_INT2_MASK (1<<BMA423x_INT2_PIN)

typedef struct bma4_accel Accel;
typedef struct bma4_accel_config Acfg;

static void bma4xx_hal_delay_usec(uint32_t period_us, void *intf_ptr);
static int8_t bma4xx_hal_i2c_bus_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
static int8_t bmi4xx_hal_i2c_bus_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);
int8_t BMA456_read_i2c(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t count);
int8_t BMA456_write_i2c(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t count);

class StableBMA
{

public:
    StableBMA();
    ~StableBMA();

    bool begin(uint8_t RTCType, uint8_t address = BMA4_I2C_ADDR_PRIMARY);  // Same as original but requires an RTCType from WatchyRTC or SmallRTC.

    void softReset();  // Same as original.
    void shutDown();   // Same as original.
    void wakeUp();     // Same as original.
    bool selfTest();   // Same as original.

    uint8_t getDirection();  // Same as original except it is orientated to show the proper higher edge on your Watchy.
    bool IsUp();             // Returns True if your Watchy is in the Tilt position (with flexible room).

    bool setAccelConfig(Acfg &cfg);    // Same as original.
    bool getAccelConfig(Acfg &cfg);    // Same as original.
    bool getAccel(Accel &acc);         // Same as original with the exception that it inverts the x and y axes on the necessary RTCType.
    bool getAccelEnable();             // Same as original.
    bool disableAccel();               // Same as original.
    bool enableAccel(bool en = true);  // Same as original.

    bool setINTPinConfig(struct bma4_int_pin_config config, uint8_t pinMap);  // Same as original.
    bool getINT();  // Same as original.
    uint8_t getIRQMASK();  // Same as original.
    bool disableIRQ(uint16_t int_map = BMA456W_STEP_CNTR_INT);  // Same as original.
    bool enableIRQ(uint16_t int_map = BMA456W_STEP_CNTR_INT);   // Same as original.
    bool isStepCounter();  // Same as original.
    bool isDoubleClick(); // Same as original.  Can be used AFTER didBMAWakeUp(wakeupBit) to determine if this is true or not.
    bool isTilt();        // Same as original.  Can be used AFTER didBMAWakeUp(wakeupBit) to determine if this is true or not.
    bool isActivity();    // Same as original.  Can be used AFTER didBMAWakeUp(wakeupBit) to determine if this is true or not.
    bool isAnyNoMotion(); // Same as original.  Can be used AFTER didBMAWakeUp(wakeupBit) to determine if this is true or not.
    bool didBMAWakeUp(uint64_t hwWakeup); // Allows you to tell via wakeupBit, if the BMA woke the Watchy, if it did, it reads the reason so you can use the above 4 functions.

    bool stepCounterWatermark(uint16_t level);
    bool resetStepCounter();  // Same as original.
    uint32_t getCounter();    // Same as original.

    float readTemperature();  // Same as original.
    float readTemperatureF(); // Fixed to allow for 0C to be an actual temperature.

    uint16_t getErrorCode();  // Same as original.
    uint16_t getStatus();     // Same as original.
    uint32_t getSensorTime(); // Same as original.

    const char *getActivity(); // Same as original.
    bool setRemapAxes(struct bma4_remap *remap_data); // Same as original.

    bool enableFeature(uint8_t feature, uint8_t enable ); // Same as original.
    bool enableStepCountInterrupt(bool en = true);        // Same as original.
    bool enableTiltInterrupt(bool en = true);             // Same as original.
    bool enableWakeupInterrupt(bool en = true);           // Same as original.
    bool enableAnyNoMotionInterrupt(bool en = true);      // Same as original.
    bool enableActivityInterrupt(bool en = true);         // Same as original.
    uint32_t WakeMask();   // Returns the necessary value to OR in the esp_sleep_enable_ext1_wakeup function to request BMA wakeups to work.
    bool defaultConfig();  // This is the default Configuration settings removed from Watchy::_bmaConfig(), corrected based on needs of RTCType.  _bmaConfig() should only consist of the begin() call and after that, the defaultConfig().
    bool enableDoubleClickWake(bool en = true); // Enables/Disables DoubleClick and the Wake Interrupt
    bool enableTiltWake(bool en = true);        // Enables/Disables Tilt and the Wake Interrupt

    static float lsb_to_ms2(int16_t val, float g_range, uint8_t bit_width);
    int8_t bma4_interface_i2c_init(struct bma4_dev *bma);

private:
    uint8_t __address;
    uint8_t __RTCTYPE;
    uint16_t __IRQ_MASK;
    bool __init;
    struct bma4_dev bma;
};
