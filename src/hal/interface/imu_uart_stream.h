/**
 * 1 kHz IMU stream on a UART, with a PPS and GPRMC output that lets a Livox
 * lidar lock its clock to the same time base as the IMU timestamps.
 *
 * Pins on the deck connector (all 3.0 V push-pull outputs):
 *
 *   TX2 (PA2)   IMU frames, 1 Mbaud 8N1      -> RXD of the USB serial adapter
 *   RX2 (PA3)   PPS, rising edge every 1 s   -> Mid-360 pin 8  (purple/white)
 *   TX1 (PC10)  GPRMC, 9600 baud 8N1         -> Mid-360 pin 10 (gray/white)
 *   GND                                      -> adapter GND and Mid-360 ground
 *
 * The frame format is described in imu_uart_proto.h.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "imu_types.h"

/**
 * Set up the UARTs and the PPS timer, and start the task that sends GPRMC.
 */
void imuUartStreamInit(void);

/**
 * Send one IMU sample. Never blocks: if the previous frame is still on the
 * wire the sample is dropped and counted in the imuStream.dropped log variable.
 *
 * @param interruptTimestampUs usecTimestamp() taken in the data ready interrupt
 * @param acc                  Acceleration [G], before low pass filtering
 * @param gyro                 Angular rate [deg/s], before low pass filtering
 * @param gyroCalibrated       True when the gyro bias has been subtracted
 */
void imuUartStreamSend(uint64_t interruptTimestampUs, const Axis3f* acc, const Axis3f* gyro, bool gyroCalibrated);
