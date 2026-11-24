#include "unity.h"
#include "num.h"


void testThatLimitUint16NotLimitInRange() {
  // Fixture
  int32_t value = 4711;
  uint16_t expected = 4711;

  // Test
  uint16_t actual = limitUint16(value);

  // Assert
  TEST_ASSERT_EQUAL_UINT16(expected, actual);
}


void testThatLimitUint16LimitZero() {
  // Fixture
  int32_t value = -100;
  uint16_t expected = 0;

  // Test
  uint16_t actual = limitUint16(value);

  // Assert
  TEST_ASSERT_EQUAL_UINT16(expected, actual);
}


void testThatLimitUint16LimitMax() {
  // Fixture
  int32_t value = UINT16_MAX + 100;
  uint16_t expected = UINT16_MAX;

  // Test
  uint16_t actual = limitUint16(value);

  // Assert
  TEST_ASSERT_EQUAL_UINT16(expected, actual);
}


void testThatConstrainDoesNotLimitInRange() {
  // Fixture
  float value = 3.45;
  float expected = value;

  // Test
  float actual = constrain(value, 0.0, 100.0);

  // Assert
  TEST_ASSERT_EQUAL_FLOAT(expected, actual);
}


void testThatConstrainLimitsMin() {
  // Fixture
  float value = -10.0;
  float min = 3.45;
  float expected = min;

  // Test
  float actual = constrain(value, min, 100.0);

  // Assert
  TEST_ASSERT_EQUAL_FLOAT(expected, actual);
}


void testThatConstrainLimitsMax() {
  // Fixture
  float value = 10.0;
  float max = 3.45;
  float expected = max;

  // Test
  float actual = constrain(value, -100.0, max);

  // Assert
  TEST_ASSERT_EQUAL_FLOAT(expected, actual);
}


void testThatDeadbandInDeadbandPositive() {
  // Fixture
  float value = 0.5;
  float threshold = 1.0;
  float expected = 0.0;

  // Test
  float actual = deadband(value, threshold);

  // Assert
  TEST_ASSERT_EQUAL_FLOAT(expected, actual);
}


void testThatDeadbandInDeadbandNegative() {
  // Fixture
  float value = -0.5;
  float threshold = 1.0;
  float expected = 0.0;

  // Test
  float actual = deadband(value, threshold);

  // Assert
  TEST_ASSERT_EQUAL_FLOAT(expected, actual);
}


void testThatDeadbandOutsideDeadbandPositive() {
  // Fixture
  float value = 2.5;
  float threshold = 1.0;
  float expected = 1.5;

  // Test
  float actual = deadband(value, threshold);

  // Assert
  TEST_ASSERT_EQUAL_FLOAT(expected, actual);
}


void testThatDeadbandOutsideDeadbandNegative() {
  // Fixture
  float value = -2.5;
  float threshold = 1.0;
  float expected = -1.5;

  // Test
  float actual = deadband(value, threshold);

  // Assert
  TEST_ASSERT_EQUAL_FLOAT(expected, actual);
}


void testThatSingle2halfRoundsUpToTheNextPowerOfTwo() {
  // Values within half a step below a power of two round up into the next exponent
  TEST_ASSERT_EQUAL_HEX16(0x4000, single2half(1.9999f));  // 2.0
  TEST_ASSERT_EQUAL_HEX16(0x3800, single2half(0.4999f));  // 0.5
  TEST_ASSERT_EQUAL_HEX16(0x4800, single2half(7.9999f));  // 8.0
  TEST_ASSERT_EQUAL_HEX16(0xC000, single2half(-1.9999f)); // -2.0
}


void testThatSingle2halfConvertsExactValues() {
  TEST_ASSERT_EQUAL_HEX16(0x3C00, single2half(1.0f));
  TEST_ASSERT_EQUAL_HEX16(0x4000, single2half(2.0f));
  TEST_ASSERT_EQUAL_HEX16(0xC100, single2half(-2.5f));
  TEST_ASSERT_EQUAL_HEX16(0x0000, single2half(0.0f));
  TEST_ASSERT_EQUAL_HEX16(0x7C00, single2half(1.0e6f));   // +inf
}


void testThatSingle2halfRoundTripIsWithinHalfAStep() {
  // Half a step of a 10-bit mantissa is at most 2^-11 of the value
  for (float value = 0.001f; value < 60000.0f; value *= 1.0001f) {
    float actual = half2single(single2half(value));
    TEST_ASSERT_FLOAT_WITHIN(value * 0.0005f, value, actual);
  }
}
