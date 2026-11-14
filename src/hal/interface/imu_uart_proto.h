/**
 * Wire format of the 1 kHz IMU UART stream and the GPRMC sentence used to time
 * sync a Livox lidar. Pure C with no hardware dependencies, so it can be
 * compiled and tested on a host machine.
 *
 * IMU frame, 40 bytes, little endian:
 *
 *   offset  size  field
 *   0       2     sync, 0xA5 0x5A
 *   2       1     version (IMU_UART_FRAME_VERSION)
 *   3       1     flags (IMU_UART_FLAG_*)
 *   4       2     seq, increments by one per IMU sample, also for samples that
 *                 were dropped on the wire, so a gap is a count of lost frames
 *   6       8     timeUs, time of the IMU data ready interrupt in microseconds
 *                 since the Unix epoch, in the time base announced on GPRMC
 *   14      12    acc x, y, z as float32 [G], not low pass filtered
 *   26      12    gyro x, y, z as float32 [deg/s], not low pass filtered
 *   38      2     CRC-16/CCITT-FALSE over bytes 2..37
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

#define IMU_UART_SYNC0 0xA5
#define IMU_UART_SYNC1 0x5A
#define IMU_UART_FRAME_VERSION 1

// The gyro bias has been found and is subtracted from the gyro values
#define IMU_UART_FLAG_GYRO_CALIBRATED (1 << 0)

typedef struct {
  uint8_t sync[2];
  uint8_t version;
  uint8_t flags;
  uint16_t seq;
  uint64_t timeUs;
  float acc[3];
  float gyro[3];
  uint16_t crc;
} __attribute__((packed)) imuUartFrame_t;

_Static_assert(sizeof(imuUartFrame_t) == 40, "IMU UART frame must be 40 bytes");

// Longest sentence imuUartBuildGprmc() can produce, including "\r\n"
#define IMU_UART_GPRMC_MAX_LEN 80

/**
 * CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, no final xor).
 */
uint16_t imuUartCrc16(const uint8_t* data, size_t length);

/**
 * Fill in sync, version and CRC. All other fields must be set by the caller.
 */
void imuUartFinalizeFrame(imuUartFrame_t* frame);

/**
 * Write a GPRMC sentence (terminated by "\r\n", no trailing zero) for the UTC
 * time given as seconds since the Unix epoch. Position, speed and course are
 * zero, only time and date carry information.
 *
 * @return Number of bytes written, 0 if the buffer is smaller than
 *         IMU_UART_GPRMC_MAX_LEN.
 */
size_t imuUartBuildGprmc(char* buffer, size_t size, uint64_t unixSeconds);
