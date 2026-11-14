/**
 * 1 kHz IMU stream on UART2 with PPS and GPRMC time sync outputs, see
 * imu_uart_stream.h for the wiring.
 *
 * Time base: TIM5 runs at 1 MHz from the same clock as the usec timer and
 * wraps once per second. Its PWM output is the PPS, so the rising edges are
 * generated in hardware without interrupt jitter. A GPRMC sentence announces
 * the time of each edge as SYNC_EPOCH_UNIX_S plus the number of edges since
 * boot. IMU timestamps are shifted from the usec timer into that same time
 * base, so lidar points and IMU samples share one clock.
 *
 * No interrupts are used. The IMU frames go out through DMA and the slow
 * GPRMC sentence is fed one byte per tick by a low priority task.
 */
#define DEBUG_MODULE "IMUUART"

#include <stdlib.h>

#include "stm32fxxx.h"

#include "FreeRTOS.h"
#include "task.h"

#include "config.h"
#include "debug.h"
#include "log.h"
#include "static_mem.h"
#include "usec_time.h"

#include "imu_uart_proto.h"
#include "imu_uart_stream.h"

// IMU frames: USART2 TX on PA2 (deck TX2), DMA1 stream 6 channel 4
#define STREAM_BAUDRATE         1000000
#define STREAM_UART             USART2
#define STREAM_UART_CLOCK       RCC_APB1Periph_USART2
#define STREAM_GPIO_PORT        GPIOA
#define STREAM_GPIO_CLOCK       RCC_AHB1Periph_GPIOA
#define STREAM_GPIO_PIN         GPIO_Pin_2
#define STREAM_GPIO_PIN_SOURCE  GPIO_PinSource2
#define STREAM_GPIO_AF          GPIO_AF_USART2
#define STREAM_DMA_STREAM       DMA1_Stream6
#define STREAM_DMA_CHANNEL      DMA_Channel_4
#define STREAM_DMA_CLOCK        RCC_AHB1Periph_DMA1
#define STREAM_DMA_FLAGS        (DMA_FLAG_TCIF6 | DMA_FLAG_HTIF6 | DMA_FLAG_TEIF6 | DMA_FLAG_DMEIF6 | DMA_FLAG_FEIF6)

// PPS: TIM5 channel 4 on PA3 (deck RX2)
#define PPS_TIMER               TIM5
#define PPS_TIMER_CLOCK         RCC_APB1Periph_TIM5
#define PPS_GPIO_PORT           GPIOA
#define PPS_GPIO_CLOCK          RCC_AHB1Periph_GPIOA
#define PPS_GPIO_PIN            GPIO_Pin_3
#define PPS_GPIO_PIN_SOURCE     GPIO_PinSource3
#define PPS_GPIO_AF             GPIO_AF_TIM5
#define PPS_PERIOD_US           1000000
#define PPS_HIGH_US             100000

// GPRMC: USART3 TX on PC10 (deck TX1)
#define GPRMC_BAUDRATE          9600
#define GPRMC_UART              USART3
#define GPRMC_UART_CLOCK        RCC_APB1Periph_USART3
#define GPRMC_GPIO_PORT         GPIOC
#define GPRMC_GPIO_CLOCK        RCC_AHB1Periph_GPIOC
#define GPRMC_GPIO_PIN          GPIO_Pin_10
#define GPRMC_GPIO_PIN_SOURCE   GPIO_PinSource10
#define GPRMC_GPIO_AF           GPIO_AF_USART3

// UTC time announced for the PPS edge at boot: 2020-01-01 00:00:00
#define SYNC_EPOCH_UNIX_S       1577836800ULL

// A PPS edge time measurement is used if the two usec timer readings around
// the TIM5 reading are at most this far apart
#define EDGE_MEASURE_MAX_SPAN_US  2
// The time base is only moved if it is off by more than the measurement noise
#define RESYNC_THRESHOLD_US       2

#define SYNC_TASK_NAME          "IMUUARTSYNC"
#define SYNC_TASK_PRI           2
#define SYNC_TASK_STACKSIZE     (2 * configMINIMAL_STACK_SIZE)

static bool isInit = false;

static uint8_t streamBuffer[sizeof(imuUartFrame_t)];
static uint16_t nextSeq;
static uint32_t droppedFrames;

// Added to a usec timer timestamp to get microseconds since the Unix epoch in
// the time base announced on GPRMC. Written by the sync task inside a critical
// section and read by the sensor task, which has a higher priority and thus
// never sees a half written value.
static int64_t syncOffsetUs;
static uint64_t lastEdgeUs;
static uint32_t ppsCount;
static uint16_t resyncCount;

STATIC_MEM_TASK_ALLOC(imuUartSyncTask, SYNC_TASK_STACKSIZE);

static void streamUartInit(void)
{
  GPIO_InitTypeDef gpio;
  USART_InitTypeDef usart;
  DMA_InitTypeDef dma;

  RCC_AHB1PeriphClockCmd(STREAM_GPIO_CLOCK | STREAM_DMA_CLOCK, ENABLE);
  RCC_APB1PeriphClockCmd(STREAM_UART_CLOCK, ENABLE);

  GPIO_StructInit(&gpio);
  gpio.GPIO_Pin = STREAM_GPIO_PIN;
  gpio.GPIO_Mode = GPIO_Mode_AF;
  gpio.GPIO_OType = GPIO_OType_PP;
  gpio.GPIO_Speed = GPIO_Speed_25MHz;
  gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(STREAM_GPIO_PORT, &gpio);
  GPIO_PinAFConfig(STREAM_GPIO_PORT, STREAM_GPIO_PIN_SOURCE, STREAM_GPIO_AF);

  USART_StructInit(&usart);
  usart.USART_BaudRate = STREAM_BAUDRATE;
  usart.USART_Mode = USART_Mode_Tx;
  USART_Init(STREAM_UART, &usart);

  DMA_DeInit(STREAM_DMA_STREAM);
  DMA_StructInit(&dma);
  dma.DMA_Channel = STREAM_DMA_CHANNEL;
  dma.DMA_PeripheralBaseAddr = (uint32_t)&STREAM_UART->DR;
  dma.DMA_Memory0BaseAddr = (uint32_t)streamBuffer;
  dma.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  dma.DMA_BufferSize = sizeof(streamBuffer);
  dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
  dma.DMA_Priority = DMA_Priority_Medium;
  DMA_Init(STREAM_DMA_STREAM, &dma);

  USART_DMACmd(STREAM_UART, USART_DMAReq_Tx, ENABLE);
  USART_Cmd(STREAM_UART, ENABLE);
}

static void gprmcUartInit(void)
{
  GPIO_InitTypeDef gpio;
  USART_InitTypeDef usart;

  RCC_AHB1PeriphClockCmd(GPRMC_GPIO_CLOCK, ENABLE);
  RCC_APB1PeriphClockCmd(GPRMC_UART_CLOCK, ENABLE);

  GPIO_StructInit(&gpio);
  gpio.GPIO_Pin = GPRMC_GPIO_PIN;
  gpio.GPIO_Mode = GPIO_Mode_AF;
  gpio.GPIO_OType = GPIO_OType_PP;
  gpio.GPIO_Speed = GPIO_Speed_2MHz;
  gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPRMC_GPIO_PORT, &gpio);
  GPIO_PinAFConfig(GPRMC_GPIO_PORT, GPRMC_GPIO_PIN_SOURCE, GPRMC_GPIO_AF);

  USART_StructInit(&usart);
  usart.USART_BaudRate = GPRMC_BAUDRATE;
  usart.USART_Mode = USART_Mode_Tx;
  USART_Init(GPRMC_UART, &usart);
  USART_Cmd(GPRMC_UART, ENABLE);
}

static void ppsTimerInit(void)
{
  GPIO_InitTypeDef gpio;
  TIM_TimeBaseInitTypeDef timeBase;
  TIM_OCInitTypeDef outputCompare;
  RCC_ClocksTypeDef clocks;

  RCC_AHB1PeriphClockCmd(PPS_GPIO_CLOCK, ENABLE);
  RCC_APB1PeriphClockCmd(PPS_TIMER_CLOCK, ENABLE);

  // Same clock derivation as the usec timer, the two must tick at the same rate.
  // The APB1 timer clock is doubled when the APB1 prescaler is not /1.
  RCC_GetClocksFreq(&clocks);
  uint32_t timerClock = clocks.PCLK1_Frequency;
  if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0) {
    timerClock *= 2;
  }

  TIM_TimeBaseStructInit(&timeBase);
  timeBase.TIM_Prescaler = (uint16_t)((timerClock / 1000000U) - 1U);
  timeBase.TIM_Period = PPS_PERIOD_US - 1;
  timeBase.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(PPS_TIMER, &timeBase);

  // PWM mode 1: high while the counter is below the pulse value, so the rising
  // edge is the counter wrapping to zero
  TIM_OCStructInit(&outputCompare);
  outputCompare.TIM_OCMode = TIM_OCMode_PWM1;
  outputCompare.TIM_OutputState = TIM_OutputState_Enable;
  outputCompare.TIM_Pulse = PPS_HIGH_US;
  outputCompare.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC4Init(PPS_TIMER, &outputCompare);

  GPIO_StructInit(&gpio);
  gpio.GPIO_Pin = PPS_GPIO_PIN;
  gpio.GPIO_Mode = GPIO_Mode_AF;
  gpio.GPIO_OType = GPIO_OType_PP;
  gpio.GPIO_Speed = GPIO_Speed_50MHz;
  gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(PPS_GPIO_PORT, &gpio);
  GPIO_PinAFConfig(PPS_GPIO_PORT, PPS_GPIO_PIN_SOURCE, PPS_GPIO_AF);

  TIM_SetCounter(PPS_TIMER, 0);
  TIM_Cmd(PPS_TIMER, ENABLE);
}

/**
 * Find the usec timer time of the most recent PPS rising edge. The PPS timer
 * counts microseconds since that edge, so the edge time is the current time
 * minus the counter. Fails if the readings were interrupted too many times.
 */
static bool measureLastEdge(uint64_t* edgeUs)
{
  for (int attempt = 0; attempt < 8; attempt++) {
    const uint64_t before = usecTimestamp();
    const uint32_t sinceEdgeUs = PPS_TIMER->CNT;
    const uint64_t after = usecTimestamp();

    if (after - before <= EDGE_MEASURE_MAX_SPAN_US) {
      *edgeUs = before - sinceEdgeUs;
      return true;
    }
  }

  return false;
}

static int64_t offsetForEdge(uint32_t edgeNumber, uint64_t edgeUs)
{
  const uint64_t edgeSyncTimeUs = (SYNC_EPOCH_UNIX_S + edgeNumber) * 1000000ULL;
  return (int64_t)(edgeSyncTimeUs - edgeUs);
}

static void handlePpsEdge(void)
{
  uint64_t edgeUs;

  if (!measureLastEdge(&edgeUs)) {
    // Keep counting, the time base itself does not need a new measurement
    ppsCount++;
    lastEdgeUs += PPS_PERIOD_US;
    return;
  }

  // Normally exactly one period has passed. Derive the number of edges from
  // the elapsed time so that a late wake up of this task does not shift the
  // time base. If the usec timer was reset the elapsed time is meaningless,
  // count one edge and let the offset below absorb the jump.
  const int64_t elapsedUs = (int64_t)(edgeUs - lastEdgeUs);
  uint32_t edges = 1;
  if (elapsedUs > PPS_PERIOD_US && elapsedUs < 3600LL * PPS_PERIOD_US) {
    edges = (uint32_t)((elapsedUs + PPS_PERIOD_US / 2) / PPS_PERIOD_US);
  }
  ppsCount += edges;
  lastEdgeUs = edgeUs;

  const int64_t offsetUs = offsetForEdge(ppsCount, edgeUs);
  if (llabs(offsetUs - syncOffsetUs) > RESYNC_THRESHOLD_US) {
    taskENTER_CRITICAL();
    syncOffsetUs = offsetUs;
    taskEXIT_CRITICAL();
    resyncCount++;
  }
}

static void imuUartSyncTask(void* param)
{
  static char sentence[IMU_UART_GPRMC_MAX_LEN];
  size_t sentenceLength = 0;
  size_t sentenceIndex = 0;
  uint32_t lastCount = PPS_TIMER->CNT;

  while (1) {
    vTaskDelay(1);

    // The counter wrapping means a PPS rising edge has just been output
    const uint32_t count = PPS_TIMER->CNT;
    if (count < lastCount) {
      handlePpsEdge();
      sentenceLength = imuUartBuildGprmc(sentence, sizeof(sentence), SYNC_EPOCH_UNIX_S + ppsCount);
      sentenceIndex = 0;
    }
    lastCount = count;

    // One byte takes 1.04 ms at 9600 baud, so one byte per tick keeps the
    // transmitter busy without ever waiting for it
    if (sentenceIndex < sentenceLength && (GPRMC_UART->SR & USART_FLAG_TXE) != 0) {
      GPRMC_UART->DR = (uint8_t)sentence[sentenceIndex++];
    }
  }
}

void imuUartStreamInit(void)
{
  if (isInit) {
    return;
  }

  streamUartInit();
  gprmcUartInit();
  ppsTimerInit();

  if (!measureLastEdge(&lastEdgeUs)) {
    lastEdgeUs = usecTimestamp() - PPS_TIMER->CNT;
  }
  syncOffsetUs = offsetForEdge(0, lastEdgeUs);

  STATIC_MEM_TASK_CREATE(imuUartSyncTask, imuUartSyncTask, SYNC_TASK_NAME, NULL, SYNC_TASK_PRI);

  isInit = true;
  DEBUG_PRINT("IMU stream on TX2 at %d baud, PPS on RX2, GPRMC on TX1\n", STREAM_BAUDRATE);
}

void imuUartStreamSend(uint64_t interruptTimestampUs, const Axis3f* acc, const Axis3f* gyro, bool gyroCalibrated)
{
  if (!isInit) {
    return;
  }

  const uint16_t seq = nextSeq++;

  // The stream is enabled for as long as the previous frame is being sent
  if (DMA_GetCmdStatus(STREAM_DMA_STREAM) != DISABLE) {
    droppedFrames++;
    return;
  }

  imuUartFrame_t* frame = (imuUartFrame_t*)streamBuffer;
  frame->flags = gyroCalibrated ? IMU_UART_FLAG_GYRO_CALIBRATED : 0;
  frame->seq = seq;
  frame->timeUs = (uint64_t)((int64_t)interruptTimestampUs + syncOffsetUs);
  for (int i = 0; i < 3; i++) {
    frame->acc[i] = acc->axis[i];
    frame->gyro[i] = gyro->axis[i];
  }
  imuUartFinalizeFrame(frame);

  DMA_ClearFlag(STREAM_DMA_STREAM, STREAM_DMA_FLAGS);
  DMA_SetCurrDataCounter(STREAM_DMA_STREAM, sizeof(streamBuffer));
  DMA_Cmd(STREAM_DMA_STREAM, ENABLE);
}

LOG_GROUP_START(imuStream)
/**
 * @brief Number of IMU frames not sent because the UART was still busy
 */
LOG_ADD(LOG_UINT32, dropped, &droppedFrames)
/**
 * @brief Number of PPS rising edges output since boot
 */
LOG_ADD(LOG_UINT32, pps, &ppsCount)
/**
 * @brief Number of times the time base was moved, should stay at zero
 */
LOG_ADD(LOG_UINT16, resync, &resyncCount)
LOG_GROUP_STOP(imuStream)
