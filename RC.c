/* BEGIN Header */
/**
 ******************************************************************************
 * \file            RC.c
 * \author          Andrea Vivani
 * \brief           Interface for reading various RemoteControl receivers
 *                  (PWM / PPM / BLE / iBUS)
 ******************************************************************************
 * \copyright
 *
 * Copyright 2024 Andrea Vivani
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the “Software”), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 ******************************************************************************
 */
/* END Header */

/* Includes ------------------------------------------------------------------*/

#include "RC.h"
#include <math.h>
#include <string.h>
#include "FreeRTOSObjects.h"
#include "basicMath.h"
#include "droneTypes.h"
#include "gpio.h"

#ifdef REMOCON_BLE
#include "hci.h"
#endif /* REMOCON_BLE */

/* Macros --------------------------------------------------------------------*/
#define IBUS_LENGTH           0x20 // Length of iBUS data: 32 bytes
#define IBUS_COMMAND40        0x40 // Command to set servo or motor speed is always 0x40
#define IBUS_COMMAND_DISCOVER 0x80 // Command discover sensor (lowest 4 bits are sensor)
#define IBUS_COMMAND_TYPE     0x90 // Command discover sensor (lowest 4 bits are sensor)
#define IBUS_COMMAND_VALUE    0xA0 // Command send sensor data (lowest 4 bits are sensor)
#define IBUS_SENSORS_NUM      5

/* Private typedefs ----------------------------------------------------------*/
#if defined(REMOCON_IBUS) && defined(configRC_USE_TELEMETRY)
typedef enum {
    IBUS_SENSOR_TYPE_NONE = 0x00,
    IBUS_SENSOR_TYPE_TEMPERATURE = 0x01,
    IBUS_SENSOR_TYPE_RPM_FLYSKY = 0x02,
    IBUS_SENSOR_TYPE_EXTERNAL_VOLTAGE = 0x03,
    IBUS_SENSOR_TYPE_CELL = 0x04,       // Avg Cell voltage
    IBUS_SENSOR_TYPE_BAT_CURR = 0x05,   // battery current A * 100
    IBUS_SENSOR_TYPE_FUEL = 0x06,       // remaining battery percentage / mah drawn otherwise or fuel level no unit!
    IBUS_SENSOR_TYPE_RPM = 0x07,        // throttle value / battery capacity
    IBUS_SENSOR_TYPE_CMP_HEAD = 0x08,   //Heading  0..360 deg, 0=north 2bytes
    IBUS_SENSOR_TYPE_CLIMB_RATE = 0x09, //2 bytes m/s *100
    IBUS_SENSOR_TYPE_COG = 0x0A, //2 bytes  Course over ground(NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. unknown max uint
    IBUS_SENSOR_TYPE_GPS_STATUS = 0x0B,     //2 bytes
    IBUS_SENSOR_TYPE_ACC_X = 0x0C,          //2 bytes m/s *100 signed
    IBUS_SENSOR_TYPE_ACC_Y = 0x0D,          //2 bytes m/s *100 signed
    IBUS_SENSOR_TYPE_ACC_Z = 0x0E,          //2 bytes m/s *100 signed
    IBUS_SENSOR_TYPE_ROLL = 0x0F,           //2 bytes deg *100 signed
    IBUS_SENSOR_TYPE_PITCH = 0x10,          //2 bytes deg *100 signed
    IBUS_SENSOR_TYPE_YAW = 0x11,            //2 bytes deg *100 signed
    IBUS_SENSOR_TYPE_VERTICAL_SPEED = 0x12, //2 bytes m/s *100
    IBUS_SENSOR_TYPE_GROUND_SPEED = 0x13,   //2 bytes m/s *100 different unit than build-in sensor
    IBUS_SENSOR_TYPE_GPS_DIST = 0x14,       //2 bytes dist from home m unsigned
    IBUS_SENSOR_TYPE_ARMED = 0x15,          //2 bytes
    IBUS_SENSOR_TYPE_FLIGHT_MODE = 0x16,    //2 bytes
    IBUS_SENSOR_TYPE_PRES = 0x41,           // Pressure
    IBUS_SENSOR_TYPE_ODO1 = 0x7C,           // Odometer1
    IBUS_SENSOR_TYPE_ODO2 = 0x7D,           // Odometer2
    IBUS_SENSOR_TYPE_SPE = 0x7E,            // Speed 2bytes km/h
    IBUS_SENSOR_TYPE_GPS_LAT = 0x80,        //4bytes signed WGS84 in degrees * 1E7
    IBUS_SENSOR_TYPE_GPS_LON = 0x81,        //4bytes signed WGS84 in degrees * 1E7
    IBUS_SENSOR_TYPE_GPS_ALT = 0x82,        //4bytes signed!!! GPS alt m*100
    IBUS_SENSOR_TYPE_ALT = 0x83,            //4bytes signed!!! Alt m*100
    IBUS_SENSOR_TYPE_ALT_MAX = 0x84,        //4bytes signed MaxAlt m*100
    IBUS_SENSOR_TYPE_ALT_FLYSKY = 0xF9,     //Altitude 2 bytes signed in m
    IBUS_SENSOR_TYPE_UNKNOWN = 0xFF
} iBusSensorType_t;

typedef struct {
    iBusSensorType_t type; // sensor type
    uint8_t length;        // data length for defined sensor (can be 2 or 4)
    int32_t value;
} iBusSensor_t;
#endif /* defined(REMOCON_IBUS) && defined(configRC_USE_TELEMETRY) */

/* Extern variables ----------------------------------------------------------*/

/* Global R/C data */
#if defined(REMOCON_BLE)
extern uint8_t RCRXBufferBLE[6];
extern int RCConnectedBLE;
#elif defined(REMOCON_IBUS)
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
#endif

/* Private variables ---------------------------------------------------------*/

#if defined(REMOCON_PWM) || defined(REMOCON_PPM) || defined(REMOCON_IBUS)
static int16_t _RCCenter[4] = {0};
#endif

/* Maximum values */
static const float _RCMaxPitchRadNorm = DEG2RAD(configPITCH_MAX_DEG) / configRC_FULLSCALE;
static const float _RCMaxRollRadNorm = DEG2RAD(configROLL_MAX_DEG) / configRC_FULLSCALE;
#ifdef configCONTROL_YAW_RATE
const float _RCMaxYawRadSNorm = DEG2RAD(configYAW_MAX_DEG_S) / configRC_FULLSCALE;
#else
const float _RCMaxYawRadStep = DEG2RAD(configYAW_MAX_DEG_STEP);
#endif

#ifdef REMOCON_IBUS
#ifdef configRC_USE_TELEMETRY
iBusSensor_t _RCiBUSsensors[IBUS_SENSORS_NUM];
#endif /* configRC_USE_TELEMETRY */
static uint8_t _RCRXBufferiBUS[IBUS_LENGTH] = {0};
#endif

/* Private functions ---------------------------------------------------------*/

#if defined(REMOCON_PWM) || defined(REMOCON_PPM) || defined(REMOCON_IBUS)
static void RC_autoCentering(RC_t* RC, uint8_t channel, int32_t value) {
    static uint8_t calibratedAxesCount = 0;
    static uint8_t sampleCount[4] = {0};
    static int32_t accumulator[4] = {0};
    if (sampleCount[channel] < 10) {
        accumulator[channel] += value;
        sampleCount[channel]++;
        if (sampleCount[channel] == 10) {
            _RCCenter[channel] = (int16_t)((float)accumulator[channel] * 0.1f);
            if (++calibratedAxesCount >= 3) {
                RC->initialized = RC_INIT;
            }
        }
    }
}
#endif

/* Read BLE input from RC */
#if defined(REMOCON_BLE)
static void RC_readBLE(RC_t* RC) {
    RC->connected = RCConnectedBLE;
    RC->rud = (RCRXBufferBLE[0] - 128) * 13;
    RC->thr = RCRXBufferBLE[1] * 13;
    RC->ail = (RCRXBufferBLE[2] - 128) * 13;
    RC->ele = (128 - RCRXBufferBLE[3]) * 13;
}
#endif /* REMOCON_BLE */

#ifdef REMOCON_IBUS
/* Returns 0 if checksum is equal, 1 otherwise */
static uint8_t RC_iBUSChecksum(uint8_t length) {
    uint16_t calculatedChecksum = 0xFFFF;
    uint16_t receivedChecksum;

    if (length > IBUS_LENGTH) {
        return 1;
    }

    for (int i = 0; i < (length - 2); i++) {
        calculatedChecksum -= _RCRXBufferiBUS[i];
    }

    receivedChecksum = _RCRXBufferiBUS[length - 1] << 8 | _RCRXBufferiBUS[length - 2]; // checksum value from ibus
    return (receivedChecksum != calculatedChecksum);
}

/* Read input from iBUS receiver */
static void RC_readiBUS(RC_t* RC) {
    uint8_t channel = 0;
    //if (_RCRXBufferiBUS[0] == IBUS_LENGTH && (RC_iBUSChecksum() == 0)) {
    if (RC_iBUSChecksum(_RCRXBufferiBUS[0]) == 0) {
        if ((_RCRXBufferiBUS[0] == IBUS_LENGTH) && (_RCRXBufferiBUS[1] == IBUS_COMMAND40)) {
            memcpy(&RC->cmd, &_RCRXBufferiBUS[2], 2 * configRC_IBUS_PPM_CHANNELS);
            for (channel = 0; channel < configRC_IBUS_PPM_CHANNELS; channel++) {
                RC->cmd[channel] <<= 2; // multiply by 4 to get to same values as with PWM / PPM
                if (channel == 2) {
                    RC->cmd[2] = (RC->cmd[2] > configRC_THR_BOTTOM) ? (RC->cmd[2] - configRC_THR_BOTTOM) : 0;
                } else if (channel < 4) {
#ifdef configRC_AUTO_CENTERING
                    // Average 10 RC data during connection as the center of RC
                    if (RC->initialized == RC_NOT_INIT) {
                        RC_autoCentering(RC, channel, RC->cmd[channel]);
                    }
                    RC->cmd[channel] -= _RCCenter[channel];
#endif
                }
            }
        }
#ifdef configRC_USE_TELEMETRY
        else {
            uint8_t addr = _RCRXBufferiBUS[1] & 0x0F;
            if (addr <= IBUS_SENSORS_NUM) {
                uint8_t txBuffer[8] = {0};
                uint16_t cksum;
                uint8_t txSize = 0;
                switch (_RCRXBufferiBUS[1] & 0xF0) {
                    case IBUS_COMMAND_DISCOVER: // 0x80, discover sensor
                        txSize = 4;
                        txBuffer[0] = 0x04;
                        txBuffer[1] = IBUS_COMMAND_DISCOVER + addr;
                        cksum = 0xFFFF - txBuffer[0] - txBuffer[1];
                        break;
                    case IBUS_COMMAND_TYPE: // 0x90, send sensor type
                        txSize = 6;
                        txBuffer[0] = 0x06;
                        txBuffer[1] = IBUS_COMMAND_TYPE + addr;
                        txBuffer[2] = _RCiBUSsensors[addr - 1].type;
                        txBuffer[3] = _RCiBUSsensors[addr - 1].length;
                        cksum = 0xFFFF - txBuffer[0] - txBuffer[1] - txBuffer[2] - txBuffer[3];
                        break;
                    case IBUS_COMMAND_VALUE: // 0xA0, send sensor data
                        txSize = 6;
                        txBuffer[0] = 0x04 + _RCiBUSsensors[addr - 1].length;
                        txBuffer[1] = IBUS_COMMAND_VALUE + addr;
                        txBuffer[2] = _RCiBUSsensors[addr - 1].value & 0xFF;
                        txBuffer[3] = (_RCiBUSsensors[addr - 1].value >> 8) & 0xFF;
                        cksum = 0xFFFF - txBuffer[0] - txBuffer[1] - txBuffer[2] - txBuffer[3];
                        if (_RCiBUSsensors[addr - 1].length == 4) {
                            txSize += 2;
                            txBuffer[4] = (_RCiBUSsensors[addr - 1].value >> 16) & 0xFF;
                            txBuffer[5] = (_RCiBUSsensors[addr - 1].value >> 24) & 0xFF;
                            cksum += -txBuffer[4] - txBuffer[5];
                        }
                        break;
                    default:
                        addr = 0; // unknown command, prevent sending chksum
                        break;
                }
                if (addr > 0) {
                    txBuffer[txSize - 2] = cksum & 0xFF;
                    txBuffer[txSize - 1] = cksum >> 8;
                    HAL_UART_Transmit(&huart2, txBuffer, txSize, 0xFFFF);
                }
            }
        }
#endif /* configRC_USE_TELEMETRY */
    } else {
        HAL_UART_DMAStop(&huart2);
        memset(_RCRXBufferiBUS, 0x00, IBUS_LENGTH);
        HAL_DMA_Init(&hdma_usart2_rx);
        //hdma_usart2_rx.Instance->M0AR = _RCRXBufferiBUS;
        HAL_UART_Receive_DMA(&huart2, _RCRXBufferiBUS, IBUS_LENGTH);
    }
}
#endif

/* Functions -----------------------------------------------------------------*/
void RC_init(RC_t* RC) {
    /* Init R/C global variables */
    RC->ail = 0;
    RC->ele = 0;
    RC->thr = 0;
    RC->rud = 0;
    RC->connected = 0;
    RC->calibration = RC_DISABLED;
    RC->arming = RC_DISABLED;
    RC_anglesReset(RC);

#ifdef configUSE_SPHERE_SENS_CORRECTION
    RC->sphereCorrection = RC_DISABLED;
#ifdef configUSE_SPHERE_OFFSET_ADJUSTMENT
    RC->sphereOffsetAdj = RC_DISABLED;
#endif
#endif

#ifdef configAUTO_ALTITUDE_CONTROL
    RC->altitude = RC_ALTITUDE_MANUAL;
#endif

#if defined(REMOCON_PWM) || defined(REMOCON_PPM) || defined(REMOCON_IBUS)
    _RCCenter[0] = configRC_AIL_MID;
    _RCCenter[1] = configRC_ELE_MID;
    _RCCenter[2] = configRC_THR_BOTTOM;
    _RCCenter[3] = configRC_RUD_MID;
#endif

#ifdef REMOCON_IBUS
#ifdef configRC_USE_TELEMETRY
    /* Configure iBUS sensors */

    /* Remaining battery percentage */
    _RCiBUSsensors[0].type = IBUS_SENSOR_TYPE_FUEL;
    _RCiBUSsensors[0].length = 2;
    _RCiBUSsensors[0].value = 0;

    /* 2 bytes m/s * 100 */
    _RCiBUSsensors[1].type = IBUS_SENSOR_TYPE_CLIMB_RATE;
    _RCiBUSsensors[1].length = 2;
    _RCiBUSsensors[1].value = 0;

    /* 2 bytes m signed */
    _RCiBUSsensors[2].type = IBUS_SENSOR_TYPE_ALT_FLYSKY;
    _RCiBUSsensors[2].length = 2;
    _RCiBUSsensors[2].value = 0;

    /* 2 bytes deg * 100 signed */
    _RCiBUSsensors[3].type = IBUS_SENSOR_TYPE_PITCH;
    _RCiBUSsensors[3].length = 2;
    _RCiBUSsensors[3].value = 0;

    /* 2 bytes deg * 100 signed */
    _RCiBUSsensors[4].type = IBUS_SENSOR_TYPE_ROLL;
    _RCiBUSsensors[4].length = 2;
    _RCiBUSsensors[4].value = 0;

#endif /* configRC_USE_TELEMETRY */
    HAL_UART_Receive_DMA(&huart2, _RCRXBufferiBUS, IBUS_LENGTH);
#endif /* REMOCON_IBUS */

#if defined(REMOCON_BLE) || !defined(configRC_AUTO_CENTERING)
    RC->initialized = RC_INIT;
#endif
}

void RC_anglesReset(RC_t* RC) {
    RC->e.x = 0;
    RC->e.y = 0;
    RC->e.z = 0;
}

void RC_read(RC_t* RC) {
#ifdef REMOCON_BLE
    /* BLE communication */
    hci_user_evt_proc();
    RC_readBLE(RC);
#elif defined(REMOCON_IBUS)
    RC_readiBUS(RC);
#endif
}

void RC_parseCommands(RC_t* RC) {
#ifndef configCONTROL_YAW_RATE
    static uint8_t recenteredCheck = 0;
#endif
    RC->e.x = CONSTRAIN(RC->ail, -configRC_FULLSCALE, configRC_FULLSCALE) * _RCMaxRollRadNorm;
    RC->e.y = CONSTRAIN(RC->ele, -configRC_FULLSCALE, configRC_FULLSCALE) * _RCMaxPitchRadNorm;
#ifdef configCONTROL_YAW_RATE
    RC->e.z = CONSTRAIN(RC->rud, -configRC_FULLSCALE, configRC_FULLSCALE) * _RCMaxYawRadSNorm;
#else
    int32_t rudder = CONSTRAIN(RC->rud, -configRC_FULLSCALE, configRC_FULLSCALE);
    if (recenteredCheck == 1) {
        if (rudder > configEULER_Z_TH) {
            RC->e.z += _RCMaxYawRadStep;
        } else if (rudder < -configEULER_Z_TH) {
            RC->e.z -= _RCMaxYawRadStep;
        }
        RC->e.z = fmodf(RC->e.z, 2 * constPI);
        if (RC->e.z < 0) {
            RC->e.z += 2 * constPI;
        }
    } else {
        /* This step to avoid unexpected behavior after initialization when rudder is all moved to the side */
        if (rudder > -configEULER_Z_TH && rudder < configEULER_Z_TH) {
            recenteredCheck = 1;
        }
    }
#endif

#ifdef REMOCON_BLE
    /* RCRXBufferBLE[4]: seek bar data*/
    /* RCRXBufferBLE[5]: additional button data
   first bit: Takeoff (0 = Land,  1 = Takeoff)
   second bit: Calibration When it changes status is active
   third bit: Arming (0 = Disarmed,  1 = Armed) */
    if (RCRXBufferBLE[5] & 0x02) {
        if (RC->calibrationLock == RC_DISABLED) {
            RC->calibration = RC_ENABLED;
            RC->calibrationLock = RC_ENABLED;
        }
    } else if (RC->calibrationLock == RC_ENABLED) {
        RC->calibrationLock = RC_DISABLED;
    }

    if (RCRXBufferBLE[5] & 0x04) {
        RC->arming = RC_ENABLED;
    } else {
        RC->arming = RC_DISABLED;
    }
#else

    /* Set connected */
    RC->connected = 1;

    /* Activate Calibration Procedure */
    if ((RC->thr == 0) && (RC->ele > configRC_MAX_THRESHOLD) && (RC->ail > configRC_MAX_THRESHOLD) && (RC->rud < -configRC_MAX_THRESHOLD)
        && (RC->initialized == RC_INIT)) {
        if (RC->calibrationLock == RC_DISABLED) {
            RC->calibration = RC_ENABLED;
            RC->calibrationLock = RC_ENABLED;
        }
    } else if (RC->calibrationLock == RC_ENABLED) {
        RC->calibrationLock = RC_DISABLED;
    }

    /* Activate Arming/Disarming */
    if ((RC->thr == 0) && (RC->ele > configRC_MAX_THRESHOLD) && (RC->ail < -configRC_MAX_THRESHOLD) && (RC->rud > configRC_MAX_THRESHOLD)
        && (RC->initialized == RC_INIT)) {
        if (RC->armingLock == RC_DISABLED) {
            RC->arming = RC_ENABLED;
            RC->armingLock = RC_ENABLED;
        }
    } else if (RC->armingLock == RC_ENABLED) {
        RC->armingLock = RC_DISABLED;
    }
#endif

#ifdef configUSE_SPHERE_SENS_CORRECTION
    // Activate Sphere Calibration Procedure
    if ((RC->thr == 0) && (RC->ele < -configRC_MAX_THRESHOLD) && (RC->ail > configRC_MAX_THRESHOLD) && (RC->rud < -configRC_MAX_THRESHOLD)
        && (RC->initialized == RC_INIT)) {
        if (RC->sphereCorrectionLock == RC_DISABLED) {
            RC->sphereCorrection = RC_ENABLED;
            RC->sphereCorrectionLock = RC_ENABLED;
        }
    } else if (RC->sphereCorrectionLock == RC_ENABLED) {
        RC->sphereCorrectionLock = RC_DISABLED;
    }

#ifdef configUSE_SPHERE_OFFSET_ADJUSTMENT
    // Activate Sphere Offset Adjustment Procedure
    if ((RC->thr == 0) && (RC->ele < -configRC_MAX_THRESHOLD) && (RC->ail < -configRC_MAX_THRESHOLD) && (RC->rud < -configRC_MAX_THRESHOLD)
        && (RC->initialized == RC_INIT)) {
        if (RC->sphereOffsetAdjLock == RC_DISABLED) {
            RC->sphereOffsetAdj = RC_ENABLED;
            RC->sphereOffsetAdjLock = RC_ENABLED;
        }
    } else if (RC->sphereOffsetAdjLock == RC_ENABLED) {
        RC->sphereOffsetAdjLock = RC_DISABLED;
    }
#endif /* configUSE_SPHERE_OFFSET_ADJUSTMENT */

#endif

#if (defined(REMOCON_PPM) || defined(REMOCON_IBUS)) && defined(configAUTO_ALTITUDE_CONTROL)
    if ((RC->configRC_ALTHOLD_CHANNEL < configRC_SW_OFF_THRESHOLD)) {
        /* Switch in up position - manual mode */
        RC->altitude = RC_ALTITUDE_MANUAL;
    } else if (RC->configRC_ALTHOLD_CHANNEL < configRC_SW_ON_THRESHOLD) {
        /* Switch in midde position, automatic mode */
        RC->altitude = RC_ALTITUDE_AUTO;
    } else {
        /* switch in down position, landing mode */
        RC->altitude = RC_ALTITUDE_LANDING;
    }
#endif

#if (defined(REMOCON_PPM) || defined(REMOCON_IBUS)) && defined(configHEADLESS_CONTROL)
    if ((RC->configRC_HEADLESS_CHANNEL < configRC_SW_OFF_THRESHOLD)) {
        /* Switch in up position - headless disabled */
        RC->headless = RC_DISABLED;
    } else {
        /* switch in down position - headless enabled */
        RC->headless = RC_ENABLED;
    }
#endif
}

#if defined(REMOCON_IBUS) && defined(configRC_USE_TELEMETRY)
void RC_iBUSUpdateSensors(sensors_t sensors, AHRS_State_t ahrs) {
    _RCiBUSsensors[0].value = CONSTRAIN((uint16_t)((float)(sensors.VBatt - 3.7f) * 2000.0f), 0, 1000);
#ifdef configAUTO_ALTITUDE_CONTROL
    _RCiBUSsensors[1].value = (uint16_t)(ahrs.altState.RoC * 100.f);
    _RCiBUSsensors[2].value = (uint16_t)ahrs.altState.alt;
#endif /* configAUTO_ALTITUDE_CONTROL */
    _RCiBUSsensors[3].value = (uint16_t)(ahrs.e.x * 100.f);
    _RCiBUSsensors[4].value = (uint16_t)(ahrs.e.y * 100.f);
    return;
}
#endif /* defined(REMOCON_IBUS) && defined(configRC_USE_TELEMETRY) */

/* Callbacks -----------------------------------------------------------------*/
#if defined(REMOCON_IBUS)
void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
    if (huart == &huart2) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(taskRC, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
#elif defined(REMOCON_PWM)
/* Read PWM input from RC */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim) {
    GPIO_TypeDef* channelPort[4] = {RC_CHANNEL1_GPIO_Port, RC_CHANNEL2_GPIO_Port, RC_CHANNEL3_GPIO_Port, RC_CHANNEL4_GPIO_Port};

    uint16_t channelPin[4] = {RC_CHANNEL1_Pin, RC_CHANNEL2_Pin, RC_CHANNEL3_Pin, RC_CHANNEL4_Pin};
    /* timer data for rising and falling edge and pulse width */
    static uint16_t riseTimer[configRC_BLE_PWM_CHANNELS] = {0}, fallTimer[configRC_BLE_PWM_CHANNELS] = {0};
    /* flag for received input capture interrupt count */
    static uint8_t pulseCounter[4] = {0};
    uint8_t channel;
    volatile uint16_t timerCounter;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    RC_t RC;

    // save the counter data
    switch (htim->Channel) {
        case HAL_TIM_ACTIVE_CHANNEL_1:
            channel = 0;
            timerCounter = htim->Instance->CCR1;
            break;
        case HAL_TIM_ACTIVE_CHANNEL_2:
            channel = 1;
            timerCounter = htim->Instance->CCR2;
            break;
        case HAL_TIM_ACTIVE_CHANNEL_3:
            channel = 2;
            timerCounter = htim->Instance->CCR3;
            break;
        case HAL_TIM_ACTIVE_CHANNEL_4:
            channel = 3;
            timerCounter = htim->Instance->CCR4;
            break;
        default: return;
    }

    xQueuePeekFromISR(mbxRC, &RC);
    // Use average of 2 periods to determine RC data
    if (HAL_GPIO_ReadPin(channelPort[channel], channelPin[channel]) == GPIO_PIN_SET) {
        riseTimer[channel] += timerCounter;
        pulseCounter[channel]++;
    } else if (HAL_GPIO_ReadPin(channelPort[channel], channelPin[channel]) == GPIO_PIN_RESET) {
        fallTimer[channel] += timerCounter;
        pulseCounter[channel]++;
    }

    if (pulseCounter[channel] >= 4) {
        if (fallTimer[channel] > riseTimer[channel]) {
            RC.cmd[channel] = (fallTimer[channel] - riseTimer[channel]) * 0.5;
        } else {
            RC.cmd[channel] = (fallTimer[channel] - riseTimer[channel] + 32768) * 0.5;
        }
        riseTimer[channel] = 0;
        fallTimer[channel] = 0;
        pulseCounter[channel] = 0;

        if (channel == 2) {
            RC.cmd[2] = (RC.cmd[2] > configRC_THR_BOTTOM) ? (RC.cmd[2] - configRC_THR_BOTTOM) : 0;
        } else {
#ifdef configRC_AUTO_CENTERING
            if (RC.initialized == RC_NOT_INIT) {
                RC_autoCentering(&RC, channel, RC.cmd[channel]);
            }
#endif
            RC.cmd[channel] -= _RCCenter[channel];
        }
    }
    xQueueOverwriteFromISR(mbxRC, &RC, &xHigherPriorityTaskWoken);
    vTaskNotifyGiveFromISR(taskRC, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
#elif defined(REMOCON_PPM)
/* Read PPM input from RC */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim) {
    static uint16_t lastTimerCounter;
    volatile uint16_t timerCounter = htim->Instance->CCR1;
    volatile int32_t pulseLength = timerCounter - lastTimerCounter;
    static uint8_t channel = 0;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    RC_t RC;
    xQueuePeekFromISR(mbxRC, &RC);

    if (timerCounter < lastTimerCounter) {
        pulseLength += 60000;
    }
    lastTimerCounter = timerCounter;

    if (pulseLength > 10000) {
        channel = 0;
        vTaskNotifyGiveFromISR(taskRC, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        return;
    }

    if (channel == 2) {
        RC.cmd[2] = (pulseLength > configRC_THR_BOTTOM) ? (pulseLength - configRC_THR_BOTTOM) : 0;
    } else if (channel < 4) {
#ifdef configRC_AUTO_CENTERING
        if (RC.initialized == RC_NOT_INIT) {
            RC_autoCentering(&RC, channel, pulseLength);
        }
#endif
        RC.cmd[channel] = pulseLength - _RCCenter[channel];
    } else {
        RC.cmd[channel] = pulseLength;
    }
    channel++;
    xQueueOverwriteFromISR(mbxRC, &RC, &xHigherPriorityTaskWoken);
    return;
}
#endif
