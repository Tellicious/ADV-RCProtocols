<p align="center"> <img src="https://github.com/Tellicious/ADV-utils/assets/9076397/3ec512f1-2de6-4226-bc07-e4bfdd686a28" width=50% height=50%> </p>

# Collection of various RC Frame Decoders/Encoders

[![Unit tests](https://github.com/Tellicious/ADV-RCProtocols/actions/workflows/runTests.yml/badge.svg?branch=main&event=push&event=workflow_dispatch)](https://github.com/Tellicious/ADV-RCProtocols/actions/workflows/runTests.yml)
[![Codecov](https://codecov.io/gh/Tellicious/ADV-RCProtocols/graph/badge.svg?token=OJG3076HXJ)](https://codecov.io/gh/Tellicious/ADV-RCProtocols)

## Libraries included:
- ***PWM:*** PWM protocol decoder
- ***PPM:*** PPM protocol decoder
- ***SymaX:*** SymaX protocol decoder/encoder
- ***iBus:*** FlySky iBUS (AFHDS 2A) protocol decoder with full support for telemetry
- ***CRSF:*** CRSF protocol decoder/encoder will full support for all frame types specified by TBS specs

## Build configuration:
### CMake build options:
| Parameter     | Default Value | Description             |
| ------------- | ------------- | ----------------------- |
| `BUILD_PPM`   | ON            | Build the PPM library   |
| `BUILD_SYMAX` | ON            | Build the SymaX library |
| `BUILD_IBUS`  | ON            | Build the iBus library  |
| `BUILD_CRSF`  | ON            | Build the CRSF library  |

## Library configuration:
Configuration options passed via `RCProtocols_COMPILE_DEFS` CMake variable

### PWM:
| Parameter                    | Type | Options/Values   | Default Value | Description                                   |
| ---------------------------- | ---- | ---------------- | ------------- | --------------------------------------------- |
| `PWM_ENABLE_STATS`           | Bool | `1`,`0`          | `0`           | Enable protocol statistics                    |
| `PWM_ENABLE_FRESHNESS_CHECK` | Bool | `1`,`0`          | `0`           | Enable packet freshness check                 |
| `PWM_MAX_CHANNELS`           | Int  | Positive integer | `4`           | Max number of channels decoded                |
| `PWM_SAMPLES_NUM`            | Int  | Positive integer | `2`           | Number of samples to be averaged, per channel |

### PPM:
| Parameter                    | Type | Options/Values   | Default Value | Description                         |
| ---------------------------- | ---- | ---------------- | ------------- | ----------------------------------- |
| `PPM_ENABLE_STATS`           | Bool | `1`,`0`          | `0`           | Enable protocol statistics          |
| `PPM_ENABLE_FRESHNESS_CHECK` | Bool | `1`,`0`          | `0`           | Enable packet freshness check       |
| `PPM_MAX_CHANNELS`           | Int  | Positive integer | `8`           | Max number of channels decoded      |
| `PPM_PACKET_END_US`          | Int  | Positive integer | `2500`        | Duration of packet end pulse, in us |

### SymaX:
| Parameter                      | Type   | Options/Values        | Default Value                                         | Description                                                 |
| ------------------------------ | ------ | --------------------- | ----------------------------------------------------- | ----------------------------------------------------------- |
| `SYMAX_CONFIG_RX`              | Define | `Enabled`, `Disabled` | `Enabled` if `SYMAX_CONFIG_TX` not defined            | Enable receiver-specific basic features                     |
| `SYMAX_CONFIG_TX`              | Define | `Enabled`, `Disabled` | `Disabled`                                            | Enable transmitter-specific basic features                  |
| `SYMAX_ENABLE_FLAGS`           | Bool   | `1`,`0`               | `1`, forced to `1` if `SYMAX_ENABLE_TRIM_DATA` is `1` | Enable flags (picture, video, etc.)                         |
| `SYMAX_ENABLE_TRIM_DATA`       | Bool   | `1`,`0`               | `0`                                                   | Enable trim-data to increase command rates                  |
| `SYMAX_ENABLE_STATS`           | Bool   | `1`,`0`               | `0`                                                   | Enable protocol statistics                                  |
| `SYMAX_ENABLE_FRESHNESS_CHECK` | Bool   | `1`,`0`               | `0`, forced to `0` if only `SYMAX_CONFIG_TX` defined  | Enable packet freshness check                               |
| `SYMAX_SKIP_PREBIND`           | Bool   | `1`,`0`               | `0`                                                   | Skip SymaX pre-bind phase (should not impact functionality) |

### iBus:
| Parameter                     | Type         | Options/Values        | Default Value                             | Description                                |
| ----------------------------- | ------------ | --------------------- | ----------------------------------------- | ------------------------------------------ |
| `IBUS_CONFIG_RX`              | Define       | `Enabled`, `Disabled` | `Enabled` if `IBUS_CONFIG_TX` not defined | Enable receiver-specific basic features    |
| `IBUS_CONFIG_TX`              | Define       | `Enabled`, `Disabled` | `Disabled`                                | Enable transmitter-specific basic features |
| `IBUS_ENABLE_TELEMETRY`       | Bool         | `1`,`0`               | `0`                                       | Enable sensors telemetry                   |
| `IBUS_ENABLE_STATS`           | Bool         | `1`,`0`               | `0`                                       | Enable protocol statistics                 |
| `IBUS_ENABLE_FRESHNESS_CHECK` | Bool         | `1`,`0`               | `0`                                       | Enable packet freshness                    |
| `IBUS_TEL_MAX_SENSORS`        | Unsigned Int | 0<x<16                | `16`                                      | Max number of sensors managed              |
| `IBUS_MAX_CHANNELS`           | Unsigned Int | 0<x<14                | `14`                                      | Max number of channels parsed              |

### CRSF:
| Parameter                            | Type   | Options/Values        | Default Value                                       | Description                                                       |
| ------------------------------------ | ------ | --------------------- | --------------------------------------------------- | ----------------------------------------------------------------- |
| `CRSF_CONFIG_RX`                     | Define | `Enabled`, `Disabled` | `Enabled` if `CRSF_CONFIG_TX` not defined           | Enable receiver-specific basic features                           |
| `CRSF_CONFIG_TX`                     | Define | `Enabled`, `Disabled` | `Disabled`                                          | Enable transmitter-specific basic features                        |
| `CRSF_ENABLE_STATS`                  | Bool   | `1`,`0`               | `0`                                                 | Enable protocol statistics                                        |
| `CRSF_ENABLE_FRESHNESS_CHECK`        | Bool   | `1`,`0`               | `0`, forced to `0` if only `CRSF_CONFIG_TX` defined | Enable packet freshness check                                     |
| `CRSF_ENABLE_ADDRESS_VALIDATION`     | Bool   | `1`,`0`               | `1`                                                 | Enable frame destination address check against valid ones         |
| `CRSF_USE_CRC_CALCULATION`           | Bool   | `1`,`0`               | `0`                                                 | Use CRC calculation instead of lookup tables (saves flash)        |
| `CRSF_TEL_ENABLE_GPS`                | Bool   | `1`,`0`               | `1`                                                 | Enable GPS frame encoding / decoding                              |
| `CRSF_TEL_ENABLE_GPS_TIME`           | Bool   | `1`,`0`               | `1`                                                 | Enable GPS Time frame encoding / decoding                         |
| `CRSF_TEL_ENABLE_GPS_EXTENDED`       | Bool   | `1`,`0`               | `1`                                                 | Enable GPS Extended frame encoding / decoding                     |
| `CRSF_TEL_ENABLE_VARIO`              | Bool   | `1`,`0`               | `1`                                                 | Enable Vario frame encoding / decoding                            |
| `CRSF_TEL_ENABLE_BATTERY_SENSOR`     | Bool   | `1`,`0`               | `1`                                                 | Enable Battery frame encoding / decoding                          |
| `CRSF_TEL_ENABLE_BAROALT_VSPEED`     | Bool   | `1`,`0`               | `1`                                                 | Enable Baro Altitude and Vertical Speed frame encoding / decoding |
| `CRSF_TEL_ENABLE_AIRSPEED`           | Bool   | `1`,`0`               | `1`                                                 | Enable Airspeed frame encoding / decoding                         |
| `CRSF_TEL_ENABLE_HEARTBEAT`          | Bool   | `1`,`0`               | `1`                                                 | Enable Heartbeat frame encoding / decoding                        |
| `CRSF_TEL_ENABLE_RPM`                | Bool   | `1`,`0`               | `1`                                                 | Enable RPM frame encoding / decoding                              |
| `CRSF_TEL_ENABLE_TEMPERATURE`        | Bool   | `1`,`0`               | `1`                                                 | Enable Temperature frame encoding / decoding                      |
| `CRSF_TEL_ENABLE_VOLTAGES`           | Bool   | `1`,`0`               | `1`                                                 | Enable Voltages frame encoding / decoding                         |
| `CRSF_TEL_ENABLE_VTX`                | Bool   | `1`,`0`               | `1`                                                 | Enable VTX frame encoding / decoding                              |
| `CRSF_TEL_ENABLE_LINK_STATISTICS`    | Bool   | `1`,`0`               | `1`                                                 | Enable Link Statistics frame encoding / decoding                  |
| `CRSF_ENABLE_RC_CHANNELS`            | Bool   | `1`,`0`               | `1`                                                 | Enable RC Channels frame encoding / decoding                      |
| `CRSF_TEL_ENABLE_LINK_STATISTICS_RX` | Bool   | `1`,`0`               | `1`                                                 | Enable RX Link Statistics frame encoding / decoding               |
| `CRSF_TEL_ENABLE_LINK_STATISTICS_TX` | Bool   | `1`,`0`               | `1`                                                 | Enable TX Link Statistics frame encoding / decoding               |
| `CRSF_TEL_ENABLE_ATTITUDE`           | Bool   | `1`,`0`               | `1`                                                 | Enable Attitude frame encoding / decoding                         |
| `CRSF_TEL_ENABLE_MAVLINK_FC`         | Bool   | `1`,`0`               | `1`                                                 | Enable MAVLink FC frame encoding / decoding                       |
| `CRSF_TEL_ENABLE_FLIGHT_MODE`        | Bool   | `1`,`0`               | `1`                                                 | Enable Flight Mode frame encoding / decoding                      |
| `CRSF_TEL_ENABLE_ESP_NOW_MESSAGES`   | Bool   | `1`,`0`               | `1`                                                 | Enable ESP Now Messages frame encoding / decoding                 |
| `CRSF_TEL_ENABLE_PARAMETER_GROUP`    | Bool   | `1`,`0`               | `1`                                                 | Enable Paramer group of frames encoding / decoding                |
| `CRSF_ENABLE_COMMAND`                | Bool   | `1`,`0`               | `1`                                                 | Enable Command frame encoding / decoding                          |
| `CRSF_TEL_ENABLE_MAVLINK_ENVELOPE`   | Bool   | `1`,`0`               | `1`                                                 | Enable MAVLink Envelope frame encoding / decoding                 |
| `CRSF_TEL_ENABLE_MAVLINK_STATUS`     | Bool   | `1`,`0`               | `1`                                                 | Enable MAVLink Status frame encoding / decoding                   |

## Usage Example

### PWM
```cpp
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim) {
    PWM_Status_t status = PWM_WAIT;
    switch (htim->Channel) {
        case HAL_TIM_ACTIVE_CHANNEL_1: status = PWM_processPacket(&_PWM, 0, HAL_GPIO_ReadPin(RC_CHANNEL1_GPIO_Port, RC_CHANNEL1_Pin), htim->Instance->CCR1); break;
        case HAL_TIM_ACTIVE_CHANNEL_2: status = PWM_processPacket(&_PWM, 1, HAL_GPIO_ReadPin(RC_CHANNEL2_GPIO_Port, RC_CHANNEL2_Pin), htim->Instance->CCR2); break;
        case HAL_TIM_ACTIVE_CHANNEL_3: status = PWM_processPacket(&_PWM, 2, HAL_GPIO_ReadPin(RC_CHANNEL3_GPIO_Port, RC_CHANNEL3_Pin), htim->Instance->CCR3); break;
        case HAL_TIM_ACTIVE_CHANNEL_4: status = PWM_processPacket(&_PWM, 3, HAL_GPIO_ReadPin(RC_CHANNEL4_GPIO_Port, RC_CHANNEL4_Pin), htim->Instance->CCR4); break;
        default: break;
    }

    if (status == PWM_SUCCESS) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(taskRC, &xHigherPriorityTaskWoken); //Process received date
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        return;
    }
}
```

### PPM
```cpp
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim) {
    if (PPM_processPacket(&_PPM, htim->Instance->CCR1) == PPM_SUCCESS) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(taskRC, &xHigherPriorityTaskWoken); //Process received date
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        return;
    }
}
```

### iBus
```cpp
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size) {
    if (huart == &huart2) {
        uint8_t txbuf[32], txlen;
        if (iBus_handleTelemetryFromISR(&_iBus, _RCRXBufferiBUS, txbuf, &txlen) == IBUS_TEL_REPLY_READY) {
            HAL_UART_Transmit(&huart2, txbuf, txlen, 0xFFFF);
        }
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(taskRC, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
```
```cpp
utilsStatus_t RC_readHW(RC_t* RC) {
    utilsStatus_t status = UTILS_STATUS_SUCCESS;
    if (iBus_processFrame(&_iBus, _RCRXBufferiBUS) == IBUS_SUCCESS) {
        memcpy(RC->cmd, _iBus.channels, configRC_CHANNELS * sizeof(uint16_t));
    } else {
        HAL_UART_DMAStop(&huart2);
        memset(_RCRXBufferiBUS, 0x00, IBUS_SERVO_FRAME_LEN);
        HAL_DMA_Init(&hdma_usart2_rx);
        status = UTILS_STATUS_ERROR;
    }
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, _RCRXBufferiBUS, IBUS_SERVO_FRAME_LEN);
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
    return status;
}
```


### CRSF
```cpp
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size) {
    if (huart == &huart2) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(taskRC, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
```
```cpp
utilsStatus_t RC_readHW(RC_t* RC) {
    utilsStatus_t status = UTILS_STATUS_SUCCESS;
#elif defined(REMOCON_CRSF)
    CRSF_FrameType_t frameType;
    if (CRSF_processFrame(&_CRSF, _RCRXBufferCRSF, &frameType) == CRSF_SUCCESS) {
        if (frameType == CRSF_FRAMETYPE_RC_CHANNELS_PACKED) {
            memcpy(RC->cmd, _CRSF.RC.channels, configRC_CHANNELS * sizeof(uint16_t));
        } else {
            status = UTILS_STATUS_WARNING; // Frame processed but not RC
        }
    } else {
        HAL_UART_DMAStop(&huart2);
        memset(_RCRXBufferCRSF, 0x00, CRSF_MAX_FRAME_LEN);
        status = UTILS_STATUS_ERROR;
    }
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, _RCRXBufferCRSF, CRSF_MAX_FRAME_LEN);
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
    return status;
}
```


## Tests coverage:
| Object | Coverage |
| ------ | -------: |
| PWM    |     100% |
| PPM    |     100% |
| SymaX  |     100% |
| iBus   |     100% |
| CRSF   |     100% |