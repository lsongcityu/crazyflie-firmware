/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * ina228.c - Deck driver for INA228 I2C power/current/voltage monitor
 */

#include <stdint.h>
#include <stdlib.h>
#include "stm32fxxx.h"

#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"
#include "system.h"
#include "deck.h"
#include "param.h"
#include "log.h"
#include "i2cdev.h"

#define INA228_I2C_ADDR         0x40

/* INA228 register map */
#define INA228_REG_CONFIG       0x00  /* 16-bit R/W */
#define INA228_REG_ADC_CONFIG   0x01  /* 16-bit R/W */
#define INA228_REG_SHUNT_CAL    0x02  /* 16-bit R/W */
#define INA228_REG_VSHUNT       0x04  /* 24-bit R   */
#define INA228_REG_VBUS         0x05  /* 24-bit R   */
#define INA228_REG_DIETEMP      0x06  /* 16-bit R   */
#define INA228_REG_CURRENT      0x07  /* 24-bit R   */
#define INA228_REG_POWER        0x08  /* 24-bit R   */
#define INA228_REG_DIAG_ALRT    0x0B  /* 16-bit R/W */
#define INA228_REG_MFR_ID       0x3E  /* 16-bit R   */
#define INA228_REG_DEVICE_ID    0x3F  /* 16-bit R   */

#define INA228_CFG_RST          (1u << 15)
#define INA228_MFR_ID_EXPECTED  0x5449  /* "TI" */

/*
 * Calibration for the Adafruit INA228 breakout (15 mOhm shunt, ADCRANGE = 0):
 *   CURRENT_LSB = 10 A / 2^19 ≈ 19.073 uA
 *   SHUNT_CAL   = 13107.2e6 * CURRENT_LSB * R_SHUNT ≈ 3750
 *   POWER_LSB   = 3.2 * CURRENT_LSB ≈ 61.03 uW
 *   VBUS_LSB    = 195.3125 uV   (fixed)
 *   TEMP_LSB    = 7.8125 m°C    (fixed)
 */
#define INA228_R_SHUNT       0.015f
#define INA228_MAX_CURRENT   10.0f
#define INA228_CURRENT_LSB   (INA228_MAX_CURRENT / 524288.0f)
#define INA228_SHUNT_CAL     3750u
#define INA228_VBUS_LSB      195.3125e-6f
#define INA228_POWER_LSB     (3.2f * INA228_CURRENT_LSB)
#define INA228_TEMP_LSB      7.8125e-3f

/*
 * ADC configuration word (optimised for >=100 Hz dynamic capture):
 *   MODE   = 0xF  (continuous bus + shunt + temperature)
 *   VBUSCT = 3    (280 us  — ~15-bit effective, good precision)
 *   VSHCT  = 3    (280 us  — ~15-bit effective for current)
 *   VTCT   = 0    (50 us   — temperature needs no high precision)
 *   AVG    = 0    (1 sample, no hardware averaging)
 *
 * Total conversion cycle = 280 + 280 + 50 = 610 us  →  ~1640 Hz sensor output.
 * The task reads at 1 kHz so every log sample at 100 Hz gets fresh data.
 */
#define INA228_ADC_CFG_VAL   0xF6C0u


static bool isInit;

static float vBus;
static float iShunt;
static float power;
static float dieTemp;
static int16_t vBusMV;
static int16_t iShuntMA;
static int16_t powerMW;

static void ina228Task(void *prm);

/* ---- I2C helpers (INA228 transmits MSB-first) ---- */

static bool ina228Read16(uint8_t reg, uint16_t *val)
{
  uint8_t buf[2];
  bool ok = i2cdevReadReg8(I2C1_DEV, INA228_I2C_ADDR, reg, 2, buf);
  if (ok) {
    *val = ((uint16_t)buf[0] << 8) | buf[1];
  }
  return ok;
}

static bool ina228Read24(uint8_t reg, uint32_t *val)
{
  uint8_t buf[3];
  bool ok = i2cdevReadReg8(I2C1_DEV, INA228_I2C_ADDR, reg, 3, buf);
  if (ok) {
    *val = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
  }
  return ok;
}

static bool ina228Write16(uint8_t reg, uint16_t val)
{
  uint8_t buf[2];
  buf[0] = (val >> 8) & 0xFF;
  buf[1] = val & 0xFF;
  return i2cdevWriteReg8(I2C1_DEV, INA228_I2C_ADDR, reg, 2, buf);
}

static int32_t signExtend20(uint32_t v)
{
  if (v & 0x80000) {
    return (int32_t)(v | 0xFFF00000);
  }
  return (int32_t)v;
}

/* ---- Deck callbacks ---- */

static void ina228Init(DeckInfo *info)
{
  if (isInit) {
    return;
  }

  uint8_t dummy = 0;
  if (!i2cdevWrite(I2C1_DEV, INA228_I2C_ADDR, 1, &dummy)) {
    DEBUG_PRINT("INA228 I2C [FAIL]\n");
    return;
  }

  uint16_t mfrId = 0;
  uint16_t devId = 0;
  ina228Read16(INA228_REG_MFR_ID, &mfrId);
  ina228Read16(INA228_REG_DEVICE_ID, &devId);

  if (mfrId != INA228_MFR_ID_EXPECTED) {
    DEBUG_PRINT("INA228 bad MFR ID: 0x%04X\n", mfrId);
    return;
  }
  DEBUG_PRINT("INA228 I2C [OK] MFR:0x%04X DEV:0x%04X\n", mfrId, devId);

  ina228Write16(INA228_REG_CONFIG, INA228_CFG_RST);
  vTaskDelay(M2T(2));

  ina228Write16(INA228_REG_ADC_CONFIG, INA228_ADC_CFG_VAL);
  ina228Write16(INA228_REG_SHUNT_CAL, INA228_SHUNT_CAL);

  xTaskCreate(ina228Task, "ina228",
              2 * configMINIMAL_STACK_SIZE, NULL,
              2, NULL);

  isInit = true;
}

static void ina228Task(void *prm)
{
  systemWaitStart();

  TickType_t lastWakeTime = xTaskGetTickCount();

  while (1) {
    vTaskDelayUntil(&lastWakeTime, M2T(1));

    uint32_t rawVbus = 0;
    uint32_t rawCur = 0;
    uint32_t rawPwr = 0;
    uint16_t rawTemp = 0;

    ina228Read24(INA228_REG_VBUS, &rawVbus);
    ina228Read24(INA228_REG_CURRENT, &rawCur);
    ina228Read24(INA228_REG_POWER, &rawPwr);
    ina228Read16(INA228_REG_DIETEMP, &rawTemp);

    /* VBUS [23:4]: unsigned 20-bit, LSB = 195.3125 uV */
    vBus = (float)(rawVbus >> 4) * INA228_VBUS_LSB;

    /* CURRENT [23:4]: signed 20-bit, LSB = CURRENT_LSB */
    iShunt = (float)signExtend20(rawCur >> 4) * INA228_CURRENT_LSB;

    /* POWER [23:0]: unsigned 24-bit, LSB = 3.2 * CURRENT_LSB */
    power = (float)rawPwr * INA228_POWER_LSB;

    /* DIETEMP [15:4]: signed 12-bit (arithmetic shift preserves sign) */
    dieTemp = (float)((int16_t)rawTemp >> 4) * INA228_TEMP_LSB;

    vBusMV = (int16_t)(vBus * 1000.0f);
    iShuntMA = (int16_t)(iShunt * 1000.0f);
    powerMW = (int16_t)(power * 1000.0f);
  }
}

static const DeckDriver ina228_deck = {
  .vid = 0x00,
  .pid = 0x00,
  .name = "bcINA228",

  .usedPeriph = DECK_USING_I2C,

  .init = ina228Init,
};

DECK_DRIVER(ina228_deck);

PARAM_GROUP_START(deck)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcINA228, &isInit)
PARAM_GROUP_STOP(deck)

LOG_GROUP_START(ina228)
LOG_ADD(LOG_FLOAT, v, &vBus)
LOG_ADD(LOG_INT16, v_mV, &vBusMV)
LOG_ADD(LOG_FLOAT, i, &iShunt)
LOG_ADD(LOG_INT16, i_mA, &iShuntMA)
LOG_ADD(LOG_FLOAT, p, &power)
LOG_ADD(LOG_INT16, p_mW, &powerMW)
LOG_ADD(LOG_FLOAT, temp, &dieTemp)
LOG_GROUP_STOP(ina228)
