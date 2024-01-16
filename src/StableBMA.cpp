#include "StableBMA.h"

/* Forked from bma.cpp by GuruSR (https://www.github.com/GuruSR/StableBMA)
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
 * bma.cpp - Arduino library for Bosch BMA423 accelerometer software library.
 * Created by Lewis He on July 27, 2020.
 * github:https://github.com/lewisxhe/BMA423_Library
 */

#define DEBUGPORT Serial
#ifdef DEBUGPORT
#define DEBUG(...) DEBUGPORT.printf(__VA_ARGS__)
#else
#define DEBUG(...)
#endif
#include <Arduino.h>

StableBMA::StableBMA()
{
    __init = false;
}

StableBMA::~StableBMA() {}

bool StableBMA::begin(uint8_t RTCType, uint8_t address)
{
    uint8_t rslt = 0;
    if (__init || RTCType == 0)
    {
        return true;
    }

    __RTCTYPE = RTCType;
    bma.intf = BMA4_I2C_INTF;
    bma.variant = BMA45X_VARIANT;

    if (bma4_interface_i2c_init(&bma) != BMA4_OK)
    {
        DEBUG("BMA456 INTERFACE FAIL\n");
        return false;
    }

    if (bma456w_init(&bma) != BMA4_OK)
    {
        DEBUG("BMA456 INIT FAIL\n");
        return false;
    }

    softReset();
    
    if (bma456w_write_config_file(&bma) != BMA4_OK)
    {
        DEBUG("BMA456 WRITE CONFIG FAIL\n");
        return false;
    }

    bma4_delay_us_fptr_t(20);

    __init = true;

    DEBUG("BMA456 SUCCESS\n");

    return __init;
}

void StableBMA::softReset()
{
    bma4_soft_reset(&bma);
}

void StableBMA::shutDown()
{
    bma4_set_advance_power_save(BMA4_DISABLE, &bma);
}

void StableBMA::wakeUp()
{
    bma4_set_advance_power_save(BMA4_ENABLE, &bma);
}

uint16_t StableBMA::getErrorCode()
{
    struct bma4_err_reg err;
    uint16_t rslt = bma4_get_error_status(&err, &bma);
    return rslt;
}

uint16_t StableBMA::getStatus()
{
    uint16_t status;
    bma4_read_int_status(&status, &bma);
    return status;
}

uint32_t StableBMA::getSensorTime()
{
    uint32_t ms;
    bma4_get_sensor_time(&ms, &bma);
    return ms;
}

bool StableBMA::selfTest()
{
    int8_t rslt = BMA4_SELFTEST_FAIL;
    bma4_perform_accel_selftest(&rslt, &bma);
    return (rslt == BMA4_SELFTEST_PASS);
}

uint8_t StableBMA::getDirection()
{
    Accel acc;
    if (!getAccel(acc))
        return 0;
    uint16_t absX = abs(acc.x);
    uint16_t absY = abs(acc.y);
    uint16_t absZ = abs(acc.z);

    if ((absZ > absX) && (absZ > absY))
    {
        return ((acc.z < 0) ? DIRECTION_DISP_DOWN : DIRECTION_DISP_UP);
    }
    else if ((absY > absX) && (absY > absZ))
    {
        return ((acc.y < 0) ? DIRECTION_BOTTOM_EDGE_UP : DIRECTION_TOP_EDGE_UP);
    }
    else
    {
        return ((acc.x < 0) ? DIRECTION_LEFT_EDGE_UP : DIRECTION_RIGHT_EDGE_UP);
    }
}

bool StableBMA::IsUp()
{
    Accel acc;
    if (!getAccel(acc))
        return false;
    // TODO check if this works now
    return (acc.x <= 0 && acc.x >= -700 && acc.y >= -300 && acc.y <= 300 && acc.z <= -750 && acc.z >= -1070);
}

float StableBMA::readTemperature()
{
    int32_t data = 0;
    bma4_get_temperature(&data, BMA4_DEG, &bma);
    float res = (float)data / (float)BMA4_SCALE_TEMP;
    /* 0x80 - temp read from the register and 23 is the ambient temp added.
     * If the temp read from register is 0x80, it means no valid
     * information is available */
    if (((data - 23) / BMA4_SCALE_TEMP) == 0x80)
    {
        res = 0;
    }
    return res;
}

float StableBMA::readTemperatureF()
{
    int32_t data = 0;
    bma4_get_temperature(&data, BMA4_DEG, &bma);
    float temp = (float)data / (float)BMA4_SCALE_TEMP;
    if (((data - 23) / BMA4_SCALE_TEMP) == 0x80)
        return 0;
    return (temp * 1.8 + 32.0);
}

bool StableBMA::getAccel(Accel &acc)
{
    memset(&acc, 0, sizeof(acc));
    if (bma4_read_accel_xyz(&acc, &bma) != BMA4_OK)
    {
        return false;
    }
    // BMA456 is always same RTC
    // if (__RTCTYPE != 1) { acc.x = -acc.x; acc.y = -acc.y; }
    return true;
}

bool StableBMA::getAccelEnable()
{
    uint8_t en;
    bma4_get_accel_enable(&en, &bma);
    return en;
}

bool StableBMA::disableAccel()
{
    return enableAccel(false);
}

bool StableBMA::enableAccel(bool en)
{
    return (BMA4_OK == bma4_set_accel_enable(en ? BMA4_ENABLE : BMA4_DISABLE, &bma));
}

bool StableBMA::setAccelConfig(Acfg &cfg)
{
    return (BMA4_OK == bma4_set_accel_config(&cfg, &bma));
}

bool StableBMA::getAccelConfig(Acfg &cfg)
{
    return (BMA4_OK == bma4_get_accel_config(&cfg, &bma));
}

bool StableBMA::setRemapAxes(struct bma4_remap *remap_data)
{
    return (BMA4_OK == bma456w_set_remap_axes(remap_data, &bma));
}

bool StableBMA::stepCounterWatermark(uint16_t level)
{
    return (BMA4_OK == bma456w_step_counter_set_watermark(level, &bma));
}

bool StableBMA::resetStepCounter()
{
    return BMA4_OK == bma456w_reset_step_counter(&bma);
}

uint32_t StableBMA::getCounter()
{
    uint32_t stepCount;
    if (bma456w_step_counter_output(&stepCount, &bma) == BMA4_OK)
    {
        return stepCount;
    }
    return 0;
}

bool StableBMA::setINTPinConfig(struct bma4_int_pin_config config, uint8_t pinMap)
{
    return BMA4_OK == bma4_set_int_pin_config(&config, pinMap, &bma);
}

bool StableBMA::getINT()
{
    return bma456w_read_int_status(&__IRQ_MASK, &bma) == BMA4_OK;
}

uint8_t StableBMA::getIRQMASK()
{
    return __IRQ_MASK;
}

bool StableBMA::disableIRQ(uint16_t int_map)
{
    return (BMA4_OK == bma456w_map_interrupt(BMA4_INTR1_MAP, int_map, BMA4_DISABLE, &bma));
}

bool StableBMA::enableIRQ(uint16_t int_map)
{
    return (BMA4_OK == bma456w_map_interrupt(BMA4_INTR1_MAP, int_map, BMA4_ENABLE, &bma));
}

bool StableBMA::enableFeature(uint8_t feature, uint8_t enable)
{
    if ((feature & BMA456W_STEP_CNTR) == BMA456W_STEP_CNTR)
    {
        //bma456w_step_detector_enable(enable ? BMA4_ENABLE : BMA4_DISABLE, &bma);
        int8_t success = bma4_set_accel_enable(BMA4_ENABLE, &bma) == BMA4_OK;
        if (success)
            success = bma456w_feature_enable(BMA456W_STEP_CNTR, BMA4_ENABLE, &bma) == BMA4_OK;
        if (success)
            success = bma456w_map_interrupt(BMA4_INTR1_MAP, BMA456W_STEP_CNTR_INT, BMA4_ENABLE, &bma) == BMA4_OK;
        return success;
    
    }
    return (BMA4_OK == bma456w_feature_enable(feature, enable, &bma));
}

bool StableBMA::isStepCounter()
{
    return (bool)(BMA456W_STEP_CNTR_INT & __IRQ_MASK);
}

bool StableBMA::isDoubleClick()
{
    return false;
    // return (bool)(BMA423_WAKEUP_INT & __IRQ_MASK);
}

bool StableBMA::isTilt()
{
    return (bool)(BMA456W_WRIST_WEAR_WAKEUP_INT & __IRQ_MASK);
}

bool StableBMA::isActivity()
{
    return (bool)(BMA456W_ACTIVITY_INT & __IRQ_MASK);
}

bool StableBMA::isAnyNoMotion()
{
    return (bool)(BMA456W_ANY_MOT_INT & __IRQ_MASK);
}

bool StableBMA::didBMAWakeUp(uint64_t hwWakeup)
{
    // This can stay BMA432 since its pin dependend of the ESP32
    bool B = ((hwWakeup & BMA423x_INT2_MASK) || (hwWakeup & BMA423x_INT2_MASK));
    if (!B)
        return B;
    if (getINT())
        return B;
    return false;
}

bool StableBMA::enableStepCountInterrupt(bool en)
{
    return (BMA4_OK == bma456w_map_interrupt(BMA4_INTR1_MAP, BMA456W_STEP_CNTR_INT, en, &bma));
}

bool StableBMA::enableTiltInterrupt(bool en)
{
    return (BMA4_OK == bma456w_map_interrupt(BMA4_INTR1_MAP, BMA456W_WRIST_WEAR_WAKEUP_INT, en, &bma));
}

bool StableBMA::enableWakeupInterrupt(bool en)
{
    return (BMA4_OK == bma456w_map_interrupt(BMA4_INTR1_MAP, BMA456W_WRIST_WEAR_WAKEUP_INT, en, &bma));
}

bool StableBMA::enableAnyNoMotionInterrupt(bool en)
{
    return (BMA4_OK == bma456w_map_interrupt(BMA4_INTR1_MAP, BMA456W_ANY_MOT_INT, en, &bma));
}

bool StableBMA::enableActivityInterrupt(bool en)
{
    return (BMA4_OK == bma456w_map_interrupt(BMA4_INTR1_MAP, BMA456W_ACTIVITY_INT, en, &bma));
}

const char *StableBMA::getActivity()
{
    uint8_t activity;
    bma456w_activity_output(&activity, &bma);
    if (activity & BMA456W_USER_STATIONARY)
    {
        return "BMA423_USER_STATIONARY";
    }
    else if (activity & BMA456W_USER_WALKING)
    {
        return "BMA423_USER_WALKING";
    }
    else if (activity & BMA456W_USER_RUNNING)
    {
        return "BMA423_USER_RUNNING";
    }
    else if (activity & BMA456W_STATE_INVALID)
    {
        return "BMA423_STATE_INVALID";
    }
    return "None";
}

uint32_t StableBMA::WakeMask()
{
    return (BMA423x_INT1_MASK | BMA423x_INT2_MASK);
}

bool StableBMA::defaultConfig()
{
    struct bma4_int_pin_config config;
    config.edge_ctrl = BMA4_LEVEL_TRIGGER;
    config.lvl = BMA4_ACTIVE_HIGH;
    config.od = BMA4_PUSH_PULL;
    config.output_en = BMA4_OUTPUT_ENABLE;
    config.input_en = BMA4_INPUT_DISABLE;

    //    if (bma4_set_int_pin_config(&config, BMA4_INTR1_MAP, &bma) != BMA4_OK) {
    //        DEBUG("BMA423 DEF CFG FAIL\n");
    //        return false;
    //    }

    Acfg cfg;
    cfg.odr = BMA4_OUTPUT_DATA_RATE_100HZ;
    cfg.range = BMA4_ACCEL_RANGE_2G;
    cfg.bandwidth = BMA4_ACCEL_NORMAL_AVG4;
    cfg.perf_mode = BMA4_CONTINUOUS_MODE;
    if (setAccelConfig(cfg))
    {
        if (enableAccel())
        {
            setINTPinConfig(config, BMA4_INTR1_MAP);

            struct bma4_remap remap_data;
            remap_data.x = BMA4_Y;
            remap_data.y = BMA4_X;
            remap_data.z = BMA4_NEG_Z;
            return setRemapAxes(&remap_data);
        }
    }
    return false;
}

bool StableBMA::enableDoubleClickWake(bool en)
{
    // if (enableFeature(BMA423_WAKEUP,en)) return enableWakeupInterrupt(en);
    return false;
}

bool StableBMA::enableTiltWake(bool en)
{
    // if (enableFeature(BMA423_TILT,en)) return enableTiltInterrupt(en);
    return false;
}

void bma4xx_hal_delay_usec(uint32_t period_us, void *intf_ptr)
{
    delayMicroseconds(period_us);
}

/*! This API is used to perform I2C read operation with sensor */
int8_t bma4xx_hal_i2c_bus_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    int8_t rslt = 0;
    // uint8_t dev_id = 0x68;
    uint8_t *dev_id = (uint8_t *)intf_ptr;

    rslt = BMA456_read_i2c(*dev_id, reg_addr, reg_data, length);

    return rslt;
}

/*! This API is used to perform I2C write operations with sensor */
int8_t bmi4xx_hal_i2c_bus_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    int8_t rslt = 0;
    //    uint8_t dev_id = 0x68;

    uint8_t *dev_id = (uint8_t *)intf_ptr;
    rslt = BMA456_write_i2c(*dev_id, reg_addr, (uint8_t *)reg_data, length);

    return rslt;
}

int8_t BMA456_read_i2c(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t count)
{
    /* dev_addr: I2C device address.
      reg_addr: Starting address for writing the data.
      reg_data: Data to be written.
      count: Number of bytes to write */
    // Begin I2C communication with provided I2C address
    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);
    // Done writting, end the transmission
    int8_t returned = Wire.endTransmission();

    if (returned)
    {
        /*
          case 1:Data too long to fit in transmit buffer
              break;
          case 2:received NACK on transmit of address.
              break;
          case 3:received NACK on transmit of data."
              break;
          case 4:Unspecified error.
              break;
          default:Unexpected Wire.endTransmission() return code:
        */
        return returned;
    }

    // Requests the required number of bytes from the sensor
    Wire.requestFrom((int)dev_addr, (int)count);

    uint16_t i;
    // Reads the requested number of bytes into the provided array
    for (i = 0; (i < count) && Wire.available(); i++)
    {
        reg_data[i] = Wire.read(); // This is for the modern Wire library
    }

    // This must return 0 on success, any other value will be interpreted as a communication failure.
    return 0;
}

int8_t BMA456_write_i2c(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t count)
{
    /*  dev_addr: I2C device address.
      reg_addr: Starting address for reading the data.
      reg_data: Buffer to take up the read data.
      count: Number of bytes to read. */
    // Begin I2C communication with provided I2C address
    Wire.beginTransmission(dev_addr);
    Wire.write(reg_addr);

    uint16_t i;
    // Writes the requested number of bytes from the provided array
    for (i = 0; i < count; i++)
    {
        Wire.write(reg_data[i]); // This is for the modern Wire library
    }
    // Done writting, end the transmission
    int8_t returned = Wire.endTransmission();
    /*
        case 1:Data too long to fit in transmit buffer
        case 2:received NACK on transmit of address.
        case 3:received NACK on transmit of data.
        case 4:Unspecified error.
        default:Unexpected Wire.endTransmission() return code:
    */
    // This must return 0 on sucess, any other value will be interpretted as a communication failure.
    return returned;
}

int8_t StableBMA::bma4_interface_i2c_init(struct bma4_dev *bma)
{
    int8_t rslt = BMA4_OK;

    if (bma != NULL)
    {

        /* Bus configuration : I2C */
        __address = BMA4_I2C_ADDR_PRIMARY;
        bma->intf = BMA4_I2C_INTF;
        bma->bus_read = bma4xx_hal_i2c_bus_read;
        bma->bus_write = bmi4xx_hal_i2c_bus_write;

        /* Assign device address to interface pointer */
        bma->intf_ptr = &__address;

        /* Assign Variant */
        bma->variant = BMA45X_VARIANT;

        /* Configure delay in microseconds */
        bma->delay_us = bma4xx_hal_delay_usec;

        /* Configure max read/write length (in bytes) (Supported length depends on target machine) */
        bma->read_write_len = 16;

        /* Set Performance mode status */
        bma->perf_mode_status = BMA4_DISABLE;
    }
    else
    {
        rslt = BMA4_E_NULL_PTR;
    }

    return rslt;
}
