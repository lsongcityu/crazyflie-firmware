/**
 * Wire format helpers for the IMU UART stream, see imu_uart_proto.h.
 */
#include "imu_uart_proto.h"

uint16_t imuUartCrc16(const uint8_t* data, size_t length)
{
  uint16_t crc = 0xFFFF;

  for (size_t i = 0; i < length; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (int bit = 0; bit < 8; bit++) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
  }

  return crc;
}

void imuUartFinalizeFrame(imuUartFrame_t* frame)
{
  frame->sync[0] = IMU_UART_SYNC0;
  frame->sync[1] = IMU_UART_SYNC1;
  frame->version = IMU_UART_FRAME_VERSION;

  const uint8_t* bytes = (const uint8_t*)frame;
  const size_t crcStart = offsetof(imuUartFrame_t, version);
  const size_t crcEnd = offsetof(imuUartFrame_t, crc);
  frame->crc = imuUartCrc16(&bytes[crcStart], crcEnd - crcStart);
}

static char* putString(char* p, const char* s)
{
  while (*s) {
    *p++ = *s++;
  }
  return p;
}

static char* putTwoDigits(char* p, unsigned int value)
{
  *p++ = (char)('0' + (value / 10) % 10);
  *p++ = (char)('0' + value % 10);
  return p;
}

static char hexDigit(unsigned int value)
{
  return (char)(value < 10 ? '0' + value : 'A' + (value - 10));
}

size_t imuUartBuildGprmc(char* buffer, size_t size, uint64_t unixSeconds)
{
  if (size < IMU_UART_GPRMC_MAX_LEN) {
    return 0;
  }

  const uint32_t secondOfDay = (uint32_t)(unixSeconds % 86400);
  const unsigned int hour = secondOfDay / 3600;
  const unsigned int minute = (secondOfDay / 60) % 60;
  const unsigned int second = secondOfDay % 60;

  // Days since the Unix epoch to civil date (Howard Hinnant's algorithm)
  const uint64_t z = unixSeconds / 86400 + 719468;
  const uint64_t era = z / 146097;
  const unsigned int dayOfEra = (unsigned int)(z - era * 146097);
  const unsigned int yearOfEra = (dayOfEra - dayOfEra / 1460 + dayOfEra / 36524 - dayOfEra / 146096) / 365;
  const unsigned int dayOfYear = dayOfEra - (365 * yearOfEra + yearOfEra / 4 - yearOfEra / 100);
  const unsigned int monthIndex = (5 * dayOfYear + 2) / 153;
  const unsigned int day = dayOfYear - (153 * monthIndex + 2) / 5 + 1;
  const unsigned int month = monthIndex < 10 ? monthIndex + 3 : monthIndex - 9;
  const unsigned int year = (unsigned int)(yearOfEra + era * 400) + (month <= 2 ? 1 : 0);

  char* p = buffer;
  p = putString(p, "$GPRMC,");
  p = putTwoDigits(p, hour);
  p = putTwoDigits(p, minute);
  p = putTwoDigits(p, second);
  p = putString(p, ".00,A,0000.0000,N,00000.0000,E,0.0,0.0,");
  p = putTwoDigits(p, day);
  p = putTwoDigits(p, month);
  p = putTwoDigits(p, year % 100);
  p = putString(p, ",0.0,E,A");

  // The checksum is the xor of everything between '$' and '*'
  uint8_t checksum = 0;
  for (const char* c = buffer + 1; c < p; c++) {
    checksum ^= (uint8_t)*c;
  }

  *p++ = '*';
  *p++ = hexDigit(checksum >> 4);
  *p++ = hexDigit(checksum & 0x0F);
  *p++ = '\r';
  *p++ = '\n';

  return (size_t)(p - buffer);
}
