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
 * mg902.c - Deck driver for the MicoAir MG-902 GPS (u-blox M9, UBX over UART1)
 *
 * Wiring (MG-902 SH1.0-6P -> Crazyflie expansion port):
 *   GND -> GND, 5V -> VCOM, Rx -> TX1, Tx -> RX1, SCL -> SCL, SDA -> SDA
 *
 * The compass is read over the deck I2C bus. Both QMC5883L (0x0D) and the
 * newer QMC5883P (0x2C), which some "5883L" modules ship with, are probed.
 * Register setup and axis mapping follow ArduPilot's AP_Compass_QMC5883L/P.
 * If no compass answers, the GPS part still runs.
 *
 * The module outputs UBX-NAV-PVT at 115200 baud by default. Once the link is
 * up, the navigation rate (default 25 Hz) and the dynamic model (default
 * airborne <4g) are set in the receiver's RAM, and NMEA output is turned off.
 * The module flash is not touched. Any other bytes on the line are skipped by
 * the frame parser. The compass mounting and calibration are parameters.
 *
 * ArduPilot switches u-blox receivers to 230400 baud and saves that to the
 * receiver's flash (GPS_SAVE_CFG defaults to 2), so a module that has been on
 * an ArduPilot flight controller no longer talks at 115200. The task therefore
 * cycles through common baud rates until it sees a valid UBX frame.
 */
#define DEBUG_MODULE "MG902"

#include <stdint.h>
#include <string.h>

#include "stm32fxxx.h"
#include "FreeRTOS.h"
#include "task.h"

#include "config.h"
#include "debug.h"
#include "system.h"
#include "deck.h"
#include "param.h"
#include "log.h"
#include "uart1.h"
#include "i2cdev.h"

/* Tried in order: MicoAir default, ArduPilot-saved, u-blox M9 factory
 * default, older u-blox default, ArduPilot moving baseline, other */
static const uint32_t baudrates[] = {115200, 230400, 38400, 9600, 460800, 57600};
#define NBR_OF_BAUDRATES        (sizeof(baudrates) / sizeof(baudrates[0]))

/* No valid UBX frame for this long: wrong baud rate, or module unplugged.
 * ArduPilot runs the receiver at 5 Hz, so this spans several PVT frames. */
#define MG902_FRAME_TIMEOUT_MS  2000
#define MG902_RX_POLL_MS        100

/* UBX frame: 0xB5 0x62 | class | id | len (LE16) | payload | ck_a ck_b */
#define UBX_SYNC1               0xB5
#define UBX_SYNC2               0x62
#define UBX_CLASS_NAV           0x01
#define UBX_ID_NAV_PVT          0x07
#define UBX_NAV_PVT_LEN         92
#define UBX_MAX_PAYLOAD         UBX_NAV_PVT_LEN
/* Longer than any message the M9 sends (NAV-SAT is ~800 bytes). A bigger
 * length means we synced on 0xB5 0x62 inside binary data, so resync at once
 * instead of swallowing several seconds of input. */
#define UBX_MAX_FRAME_LEN       1024

/* UBX-NAV-PVT flags (offset 21) */
#define UBX_PVT_FLAGS_GNSS_FIX_OK  0x01

/* Receiver configuration, UBX-CFG-VALSET written to RAM only (the module
 * flash keeps its own settings) */
#define UBX_CLASS_CFG           0x06
#define UBX_ID_CFG_VALSET       0x8A
#define UBX_CLASS_ACK           0x05
#define UBX_ID_ACK_NAK          0x00
#define UBX_ID_ACK_ACK          0x01
#define UBX_ACK_LEN             2
#define UBX_VALSET_LAYER_RAM    0x01

/* Configuration keys, the value size is encoded in bits 28-30 of the key */
#define CFG_UART1OUTPROT_NMEA          0x10740002  /* L, NMEA output on UART1 */
#define CFG_MSGOUT_UBX_NAV_PVT_UART1   0x20910007  /* U1, PVT every n epochs */
#define CFG_NAVSPG_DYNMODEL            0x20110021  /* E1, dynamic platform model */
#define CFG_RATE_MEAS                  0x30210001  /* U2, measurement period [ms] */

#define DYNMODEL_AIRBORNE_4G    8

/* QMC5883L register map */
#define QMC5883L_I2C_ADDR       0x0D
#define QMC5883L_REG_DATA       0x00  /* X, Y, Z, little-endian int16 */
#define QMC5883L_REG_STATUS     0x06
#define QMC5883L_REG_CONF1      0x09
#define QMC5883L_REG_SET_RESET  0x0B
#define QMC5883L_REG_0C         0x0C  /* reads 0x01 */
#define QMC5883L_REG_ID         0x0D  /* reads 0xFF */
/* Continuous mode | 100 Hz | 8 G range | OSR 512 */
#define QMC5883L_CONF1_VAL      0x19
#define QMC5883L_LSB_PER_GAUSS  3000.0f

/* QMC5883P register map */
#define QMC5883P_I2C_ADDR       0x2C
#define QMC5883P_REG_ID         0x00  /* reads 0x80 */
#define QMC5883P_REG_DATA       0x01  /* X, Y, Z, little-endian int16 */
#define QMC5883P_REG_SIGN       0x06
#define QMC5883P_REG_STATUS     0x09
#define QMC5883P_REG_CONF1      0x0A
#define QMC5883P_REG_CONF2      0x0B
#define QMC5883P_ID_VAL         0x80
#define QMC5883P_SIGN_VAL       0x29
/* Same values as ArduPilot: continuous mode, 100 Hz; CONF2 selects 8 G */
#define QMC5883P_CONF1_VAL      0x0B
#define QMC5883P_CONF2_VAL      0x08
#define QMC5883P_LSB_PER_GAUSS  3750.0f

#define QMC_STATUS_DRDY         0x01
/* 5883L only: a sample was overwritten before it was read. Only reading the
 * data clears it, and while it is set DRDY can stay 0, so DRDY alone can
 * stall the driver for good (seen 2026-11-01, status stuck at 0x04).
 * ArduPilot uses this bit as its data-ready test. */
#define QMC5883L_STATUS_DOR     0x04

#define MG902_MAG_PERIOD_MS     10
/* No new sample for this long: the chip dropped back to standby (power glitch
 * on the module after start-up), so it is set up again */
#define MG902_MAG_TIMEOUT_MS    500

typedef enum {
  MAG_NONE = 0,
  MAG_QMC5883L = 1,
  MAG_QMC5883P = 2,
} MagChip;

/* Module mounted flat, antenna up; direction the SH1.0 connector points */
typedef enum {
  MOUNT_CONNECTOR_TAIL = 0,
  MOUNT_CONNECTOR_RIGHT = 1,
  MOUNT_CONNECTOR_NOSE = 2,
  MOUNT_CONNECTOR_LEFT = 3,
} MagMount;

typedef enum {
  UBX_STATE_SYNC1,
  UBX_STATE_SYNC2,
  UBX_STATE_CLASS,
  UBX_STATE_ID,
  UBX_STATE_LEN1,
  UBX_STATE_LEN2,
  UBX_STATE_PAYLOAD,
  UBX_STATE_CK_A,
  UBX_STATE_CK_B,
} UbxState;

typedef struct {
  UbxState state;
  uint8_t msgClass;
  uint8_t msgId;
  uint16_t len;
  uint16_t idx;
  uint8_t ckA;
  uint8_t ckB;
  bool keep;  /* payload is stored only for NAV-PVT and ACK */
  uint8_t payload[UBX_MAX_PAYLOAD];
} UbxParser;

static bool isInit;
static UbxParser parser;

/* Decoded NAV-PVT, units converted where it helps plotting in the client */
static uint32_t iTOW;      /* GPS time of week [ms] */
static uint8_t fixType;    /* 0 none, 2 2D, 3 3D, ... */
static uint8_t fixOk;      /* gnssFixOK flag */
static uint8_t numSV;
static int32_t lon;        /* [1e-7 deg] */
static int32_t lat;        /* [1e-7 deg] */
static float hMSL;         /* [m] */
static float hAcc;         /* [m] */
static float vAcc;         /* [m] */
static float velN;         /* [m/s] */
static float velE;         /* [m/s] */
static float velD;         /* [m/s] */
static float gSpeed;       /* [m/s] */
static float headMot;      /* [deg] */
static float sAcc;         /* [m/s] */
static float pDOP;

/* Link health */
static uint32_t baudrate;
static uint8_t connected;
static uint32_t pvtCount;
static uint32_t ckErrors;
static uint32_t rxOverruns;

/* Receiver configuration, sent on every (re)connection and when a
 * parameter changes */
static uint8_t navRate = 25;                   /* [Hz] */
static uint8_t dynModel = DYNMODEL_AIRBORNE_4G;
static volatile bool cfgPending;
static uint8_t cfgAcks;
static uint8_t cfgNaks;

/* Compass, robot body frame (x forward, y left, z up), calibrated */
static uint8_t magChip;    /* MagChip */
static float magX;         /* [gauss] */
static float magY;         /* [gauss] */
static float magZ;         /* [gauss] */
static int16_t magRaw[3];  /* sensor axes, for checking the mapping */
static uint32_t magCount;
static uint32_t magErrors;
static uint8_t magStatus;   /* last status register, for diagnosis */
static uint8_t magReinits;

static uint8_t magMount = MOUNT_CONNECTOR_TAIL;  /* MagMount */

/*
 * Hard and soft iron calibration in the module frame (the body frame for
 * MOUNT_CONNECTOR_TAIL), same form as MATLAB magcal: mag_cal = (mag - b)*A.
 * Defaults: outdoor calibration 2026-11-01 (20261101_180441, module on top of
 * the frame, connector to the tail). Recalibrate after moving the module.
 */
static float magB[3] = {-0.0151f, -0.0852f, -0.0332f};
static float magA[3][3] = {
  {1.0037f, 0.0006f, 0.0054f},
  {0.0006f, 1.0048f, 0.0134f},
  {0.0054f, 0.0134f, 0.9917f},
};

static void mg902Task(void *param);
static void mg902MagTask(void *param);

/* ---- Little-endian helpers (UBX is little-endian) ---- */

static uint16_t getU2(const uint8_t *p)
{
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t getU4(const uint8_t *p)
{
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static int32_t getI4(const uint8_t *p)
{
  return (int32_t)getU4(p);
}

/* ---- UBX handling ---- */

static void decodeNavPvt(const uint8_t *p)
{
  iTOW    = getU4(&p[0]);
  fixType = p[20];
  fixOk   = p[21] & UBX_PVT_FLAGS_GNSS_FIX_OK;
  numSV   = p[23];
  lon     = getI4(&p[24]);
  lat     = getI4(&p[28]);
  hMSL    = getI4(&p[36]) * 1e-3f;
  hAcc    = getU4(&p[40]) * 1e-3f;
  vAcc    = getU4(&p[44]) * 1e-3f;
  velN    = getI4(&p[48]) * 1e-3f;
  velE    = getI4(&p[52]) * 1e-3f;
  velD    = getI4(&p[56]) * 1e-3f;
  gSpeed  = getI4(&p[60]) * 1e-3f;
  headMot = getI4(&p[64]) * 1e-5f;
  sAcc    = getU4(&p[68]) * 1e-3f;
  pDOP    = getU2(&p[76]) * 0.01f;

  pvtCount++;
}

static void decodeAck(uint8_t msgId, const uint8_t *p)
{
  /* p[0], p[1]: class and id of the message being acknowledged */
  if (p[0] == UBX_CLASS_CFG && p[1] == UBX_ID_CFG_VALSET) {
    if (msgId == UBX_ID_ACK_ACK) {
      cfgAcks++;
    } else {
      cfgNaks++;
    }
  }
}

/* Returns true when a complete frame with a valid checksum has been received */
static bool ubxParseByte(UbxParser *ps, uint8_t c)
{
  bool frameOk = false;

  /* Fletcher checksum covers class, id, length and payload */
  if (ps->state >= UBX_STATE_CLASS && ps->state <= UBX_STATE_PAYLOAD) {
    ps->ckA += c;
    ps->ckB += ps->ckA;
  }

  switch (ps->state) {
    case UBX_STATE_SYNC1:
      if (c == UBX_SYNC1) {
        ps->state = UBX_STATE_SYNC2;
      }
      break;
    case UBX_STATE_SYNC2:
      if (c == UBX_SYNC2) {
        ps->ckA = 0;
        ps->ckB = 0;
        ps->state = UBX_STATE_CLASS;
      } else {
        ps->state = (c == UBX_SYNC1) ? UBX_STATE_SYNC2 : UBX_STATE_SYNC1;
      }
      break;
    case UBX_STATE_CLASS:
      ps->msgClass = c;
      ps->state = UBX_STATE_ID;
      break;
    case UBX_STATE_ID:
      ps->msgId = c;
      ps->state = UBX_STATE_LEN1;
      break;
    case UBX_STATE_LEN1:
      ps->len = c;
      ps->state = UBX_STATE_LEN2;
      break;
    case UBX_STATE_LEN2:
      ps->len |= (uint16_t)c << 8;
      if (ps->len > UBX_MAX_FRAME_LEN) {
        ckErrors++;
        ps->state = UBX_STATE_SYNC1;
        break;
      }
      ps->idx = 0;
      ps->keep = (ps->msgClass == UBX_CLASS_NAV && ps->msgId == UBX_ID_NAV_PVT &&
                  ps->len == UBX_NAV_PVT_LEN) ||
                 (ps->msgClass == UBX_CLASS_ACK && ps->len == UBX_ACK_LEN);
      ps->state = (ps->len > 0) ? UBX_STATE_PAYLOAD : UBX_STATE_CK_A;
      break;
    case UBX_STATE_PAYLOAD:
      /* Other messages are walked through only to verify their checksum */
      if (ps->keep) {
        ps->payload[ps->idx] = c;
      }
      if (++ps->idx >= ps->len) {
        ps->state = UBX_STATE_CK_A;
      }
      break;
    case UBX_STATE_CK_A:
      if (c == ps->ckA) {
        ps->state = UBX_STATE_CK_B;
      } else {
        ckErrors++;
        ps->state = UBX_STATE_SYNC1;
      }
      break;
    case UBX_STATE_CK_B:
      if (c == ps->ckB) {
        frameOk = true;
        if (ps->keep && ps->msgClass == UBX_CLASS_NAV) {
          decodeNavPvt(ps->payload);
        } else if (ps->keep && ps->msgClass == UBX_CLASS_ACK) {
          decodeAck(ps->msgId, ps->payload);
        }
      } else {
        ckErrors++;
      }
      ps->state = UBX_STATE_SYNC1;
      break;
  }

  return frameOk;
}

/* One key per message, so a key the receiver rejects does not take the
 * others with it */
static void ubxSendValset(uint32_t key, uint32_t value)
{
  const uint8_t valueLen = (key >> 28) == 3 ? 2 : 1;  /* only U1/E1/L and U2 used */
  const uint16_t len = 8 + valueLen;
  uint8_t frame[6 + 8 + 2 + 2];
  uint8_t n = 0;

  frame[n++] = UBX_SYNC1;
  frame[n++] = UBX_SYNC2;
  frame[n++] = UBX_CLASS_CFG;
  frame[n++] = UBX_ID_CFG_VALSET;
  frame[n++] = len & 0xFF;
  frame[n++] = len >> 8;
  frame[n++] = 0;                     /* version */
  frame[n++] = UBX_VALSET_LAYER_RAM;
  frame[n++] = 0;                     /* reserved */
  frame[n++] = 0;
  for (int i = 0; i < 4; i++) {
    frame[n++] = (key >> (8 * i)) & 0xFF;
  }
  for (int i = 0; i < valueLen; i++) {
    frame[n++] = (value >> (8 * i)) & 0xFF;
  }

  uint8_t ckA = 0, ckB = 0;
  for (int i = 2; i < n; i++) {
    ckA += frame[i];
    ckB += ckA;
  }
  frame[n++] = ckA;
  frame[n++] = ckB;

  uart1SendData(n, frame);
}

static void configureReceiver(void)
{
  ubxSendValset(CFG_UART1OUTPROT_NMEA, 0);  /* NMEA would not fit at 25 Hz */
  ubxSendValset(CFG_MSGOUT_UBX_NAV_PVT_UART1, 1);
  ubxSendValset(CFG_NAVSPG_DYNMODEL, dynModel);
  ubxSendValset(CFG_RATE_MEAS, 1000 / navRate);
}

static void receiverParamChanged(void)
{
  if (navRate == 0) {
    navRate = 1;
  }
  cfgPending = true;
}

/* ---- Task ---- */

static void setBaudrate(uint32_t rate)
{
  /* uart1Init() recreates the RX queue, keep the RX interrupt off meanwhile */
  USART_ITConfig(UART1_TYPE, USART_IT_RXNE, DISABLE);
  uart1Init(rate);
  parser.state = UBX_STATE_SYNC1;
  baudrate = rate;
}

static void mg902Task(void *param)
{
  uint8_t c;
  uint8_t baudIdx = 0;

  systemWaitStart();

  TickType_t lastFrame = xTaskGetTickCount();

  while (1) {
    if (uart1GetDataWithTimeout(&c, M2T(MG902_RX_POLL_MS)) &&
        ubxParseByte(&parser, c)) {
      lastFrame = xTaskGetTickCount();
      if (!connected) {
        /* new baud rate found, or the module was power cycled */
        cfgPending = true;
      }
      connected = 1;
    }

    if (connected && cfgPending) {
      cfgPending = false;
      configureReceiver();
    }

    /* The UART1 RX queue is only 64 bytes, shorter than one NAV-PVT frame */
    if (uart1DidOverrun()) {
      rxOverruns++;
    }

    if (xTaskGetTickCount() - lastFrame > M2T(MG902_FRAME_TIMEOUT_MS)) {
      connected = 0;
      fixOk = 0;
      baudIdx = (baudIdx + 1) % NBR_OF_BAUDRATES;
      setBaudrate(baudrates[baudIdx]);
      lastFrame = xTaskGetTickCount();
    }
  }
}

/* ---- Compass ---- */

static bool qmc5883lProbe(void)
{
  uint8_t v = 0;

  /* ArduPilot: read register 0x00 once first, the ID registers are not
   * reliable before that when other devices share the bus */
  i2cdevReadByte(I2C1_DEV, QMC5883L_I2C_ADDR, QMC5883L_REG_DATA, &v);
  if (!i2cdevReadByte(I2C1_DEV, QMC5883L_I2C_ADDR, QMC5883L_REG_0C, &v) || v != 0x01) {
    return false;
  }
  if (!i2cdevReadByte(I2C1_DEV, QMC5883L_I2C_ADDR, QMC5883L_REG_ID, &v) || v != 0xFF) {
    return false;
  }

  /* 0x20/0x21 are undocumented, written as ArduPilot does */
  return i2cdevWriteByte(I2C1_DEV, QMC5883L_I2C_ADDR, QMC5883L_REG_SET_RESET, 0x01) &&
         i2cdevWriteByte(I2C1_DEV, QMC5883L_I2C_ADDR, 0x20, 0x40) &&
         i2cdevWriteByte(I2C1_DEV, QMC5883L_I2C_ADDR, 0x21, 0x01) &&
         i2cdevWriteByte(I2C1_DEV, QMC5883L_I2C_ADDR, QMC5883L_REG_CONF1, QMC5883L_CONF1_VAL);
}

static bool qmc5883pProbe(void)
{
  uint8_t v = 0;

  if (!i2cdevReadByte(I2C1_DEV, QMC5883P_I2C_ADDR, QMC5883P_REG_ID, &v) || v != QMC5883P_ID_VAL) {
    return false;
  }

  return i2cdevWriteByte(I2C1_DEV, QMC5883P_I2C_ADDR, QMC5883P_REG_SIGN, QMC5883P_SIGN_VAL) &&
         i2cdevWriteByte(I2C1_DEV, QMC5883P_I2C_ADDR, QMC5883P_REG_CONF1, QMC5883P_CONF1_VAL) &&
         i2cdevWriteByte(I2C1_DEV, QMC5883P_I2C_ADDR, QMC5883P_REG_CONF2, QMC5883P_CONF2_VAL);
}

/* Returns true when a new sample was read */
static bool magRead(void)
{
  uint8_t addr, regStatus, regData, newData;
  uint8_t status = 0;
  uint8_t buf[6];

  if (magChip == MAG_QMC5883L) {
    addr = QMC5883L_I2C_ADDR;
    regStatus = QMC5883L_REG_STATUS;
    regData = QMC5883L_REG_DATA;
    newData = QMC_STATUS_DRDY | QMC5883L_STATUS_DOR;
  } else {
    addr = QMC5883P_I2C_ADDR;
    regStatus = QMC5883P_REG_STATUS;
    regData = QMC5883P_REG_DATA;
    newData = QMC_STATUS_DRDY;
  }

  if (!i2cdevReadByte(I2C1_DEV, addr, regStatus, &status)) {
    magErrors++;
    return false;
  }
  magStatus = status;
  if (!(status & newData)) {
    return false;
  }
  if (!i2cdevReadReg8(I2C1_DEV, addr, regData, sizeof(buf), buf)) {
    magErrors++;
    return false;
  }

  for (int i = 0; i < 3; i++) {
    magRaw[i] = (int16_t)getU2(&buf[2 * i]);
  }
  const float rx = magRaw[0];
  const float ry = magRaw[1];
  const float rz = magRaw[2];

  /*
   * Module frame: x away from the connector, y to its left, z up through the
   * antenna. ArduPilot's chain, ending in its NED body frame:
   *   5883L: (-rx, ry, -rz), then YAW_90 (external), then ROLL_180 -> (-ry,  rx,  rz)
   *   5883P: ( rx, ry,  rz), then ROLL_180                          -> ( rx, -ry, -rz)
   * NED -> Crazyflie (x forward, y left, z up) negates y and z.
   *
   * Outdoor tests 2026-10-31 (MG-902, QMC5883L, connector to the tail and to
   * the nose) showed that this chain gives x = right, y = front, so x and y
   * are rotated by 90 deg on top of it. The 5883P line gets the same
   * correction but has not been checked on hardware.
   */
  float m[3];
  if (magChip == MAG_QMC5883L) {
    m[0] = -rx / QMC5883L_LSB_PER_GAUSS;
    m[1] =  ry / QMC5883L_LSB_PER_GAUSS;
    m[2] = -rz / QMC5883L_LSB_PER_GAUSS;
  } else {
    m[0] =  ry / QMC5883P_LSB_PER_GAUSS;
    m[1] = -rx / QMC5883P_LSB_PER_GAUSS;
    m[2] =  rz / QMC5883P_LSB_PER_GAUSS;
  }

  /* mag_cal = (mag - b)*A, row vector as in MATLAB */
  float c[3];
  for (int j = 0; j < 3; j++) {
    c[j] = (m[0] - magB[0]) * magA[0][j] +
           (m[1] - magB[1]) * magA[1][j] +
           (m[2] - magB[2]) * magA[2][j];
  }

  /* module frame -> robot body frame */
  switch (magMount) {
    case MOUNT_CONNECTOR_RIGHT:  /* module x points to the robot's left */
      magX = -c[1]; magY =  c[0]; break;
    case MOUNT_CONNECTOR_NOSE:   /* module x points to the tail */
      magX = -c[0]; magY = -c[1]; break;
    case MOUNT_CONNECTOR_LEFT:   /* module x points to the robot's right */
      magX =  c[1]; magY = -c[0]; break;
    default:                     /* MOUNT_CONNECTOR_TAIL: module = body */
      magX =  c[0]; magY =  c[1]; break;
  }
  magZ = c[2];

  magCount++;
  return true;
}

static void mg902MagTask(void *param)
{
  systemWaitStart();

  TickType_t lastWakeTime = xTaskGetTickCount();
  TickType_t lastSample = lastWakeTime;

  while (1) {
    vTaskDelayUntil(&lastWakeTime, M2T(MG902_MAG_PERIOD_MS));
    if (magRead()) {
      lastSample = lastWakeTime;
    } else if (lastWakeTime - lastSample > M2T(MG902_MAG_TIMEOUT_MS)) {
      if (magChip == MAG_QMC5883L) {
        qmc5883lProbe();
      } else {
        qmc5883pProbe();
      }
      magReinits++;
      lastSample = lastWakeTime;
    }
  }
}

/* ---- Deck callbacks ---- */

static void mg902Init(DeckInfo *info)
{
  if (isInit) {
    return;
  }

  DEBUG_PRINT("Enabling MG-902 GPS on UART1\n");
  setBaudrate(baudrates[0]);

  xTaskCreate(mg902Task, MG902_DECK_TASK_NAME,
              MG902_DECK_TASK_STACKSIZE, NULL, MG902_DECK_TASK_PRI, NULL);

  if (qmc5883lProbe()) {
    magChip = MAG_QMC5883L;
  } else if (qmc5883pProbe()) {
    magChip = MAG_QMC5883P;
  }

  if (magChip != MAG_NONE) {
    DEBUG_PRINT("MG-902 compass: %s\n", magChip == MAG_QMC5883L ? "QMC5883L" : "QMC5883P");
    xTaskCreate(mg902MagTask, MG902_MAG_TASK_NAME,
                MG902_MAG_TASK_STACKSIZE, NULL, MG902_MAG_TASK_PRI, NULL);
  } else {
    DEBUG_PRINT("MG-902 compass not found on I2C\n");
  }

  isInit = true;
}

static bool mg902Test()
{
  return isInit;
}

static const DeckDriver mg902_deck = {
  .vid = 0x00,
  .pid = 0x00,
  .name = "bcMG902",

  .usedGpio = 0,
  .usedPeriph = DECK_USING_UART1 | DECK_USING_I2C,

  .init = mg902Init,
  .test = mg902Test,
};

DECK_DRIVER(mg902_deck);

PARAM_GROUP_START(deck)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, bcMG902, &isInit)
PARAM_GROUP_STOP(deck)

/* Persistent: set in the client, then store to keep them over reboots */
PARAM_GROUP_START(mg902)
/* GPS navigation rate [Hz], applied right away */
PARAM_ADD_WITH_CALLBACK(PARAM_UINT8 | PARAM_PERSISTENT, navRate, &navRate, receiverParamChanged)
/* u-blox dynamic model: 0 portable, 6/7/8 airborne <1g/<2g/<4g */
PARAM_ADD_WITH_CALLBACK(PARAM_UINT8 | PARAM_PERSISTENT, dynModel, &dynModel, receiverParamChanged)
/* Compass mounting, direction of the SH1.0 connector: 0 tail, 1 right, 2 nose, 3 left */
PARAM_ADD(PARAM_UINT8 | PARAM_PERSISTENT, mount, &magMount)
/* Hard iron b [gauss], module frame */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, magBx, &magB[0])
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, magBy, &magB[1])
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, magBz, &magB[2])
/* Soft iron A, magA<row><col> = A(row,col) in mag_cal = (mag - b)*A */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, magA11, &magA[0][0])
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, magA12, &magA[0][1])
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, magA13, &magA[0][2])
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, magA21, &magA[1][0])
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, magA22, &magA[1][1])
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, magA23, &magA[1][2])
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, magA31, &magA[2][0])
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, magA32, &magA[2][1])
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, magA33, &magA[2][2])
PARAM_GROUP_STOP(mg902)

LOG_GROUP_START(mg902)
LOG_ADD(LOG_INT32, lat, &lat)
LOG_ADD(LOG_INT32, lon, &lon)
LOG_ADD(LOG_FLOAT, hMSL, &hMSL)
LOG_ADD(LOG_FLOAT, hAcc, &hAcc)
LOG_ADD(LOG_FLOAT, vAcc, &vAcc)
LOG_ADD(LOG_FLOAT, velN, &velN)
LOG_ADD(LOG_FLOAT, velE, &velE)
LOG_ADD(LOG_FLOAT, velD, &velD)
LOG_ADD(LOG_FLOAT, gSpeed, &gSpeed)
LOG_ADD(LOG_FLOAT, headMot, &headMot)
LOG_ADD(LOG_FLOAT, sAcc, &sAcc)
LOG_ADD(LOG_FLOAT, pDOP, &pDOP)
LOG_ADD(LOG_UINT8, fixType, &fixType)
LOG_ADD(LOG_UINT8, fixOk, &fixOk)
LOG_ADD(LOG_UINT8, numSV, &numSV)
LOG_ADD(LOG_UINT32, iTOW, &iTOW)
LOG_ADD(LOG_UINT32, baudrate, &baudrate)
LOG_ADD(LOG_UINT8, connected, &connected)
LOG_ADD(LOG_UINT32, pvtCount, &pvtCount)
LOG_ADD(LOG_UINT32, ckErrors, &ckErrors)
LOG_ADD(LOG_UINT32, rxOverruns, &rxOverruns)
LOG_ADD(LOG_UINT8, cfgAcks, &cfgAcks)
LOG_ADD(LOG_UINT8, cfgNaks, &cfgNaks)
LOG_ADD(LOG_UINT8, magChip, &magChip)
LOG_ADD(LOG_FLOAT, magX, &magX)
LOG_ADD(LOG_FLOAT, magY, &magY)
LOG_ADD(LOG_FLOAT, magZ, &magZ)
LOG_ADD(LOG_INT16, magRawX, &magRaw[0])
LOG_ADD(LOG_INT16, magRawY, &magRaw[1])
LOG_ADD(LOG_INT16, magRawZ, &magRaw[2])
LOG_ADD(LOG_UINT32, magCount, &magCount)
LOG_ADD(LOG_UINT32, magErrors, &magErrors)
LOG_ADD(LOG_UINT8, magStatus, &magStatus)
LOG_ADD(LOG_UINT8, magReinits, &magReinits)
LOG_GROUP_STOP(mg902)
