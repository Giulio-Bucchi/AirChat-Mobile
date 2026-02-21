/**
  ******************************************************************************
  * @file           : sx127x.h
  * @brief          : SX127x LoRa driver header
  ******************************************************************************
  */

#ifndef __SX127X_H
#define __SX127X_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* SX127x Register Addresses */
#define REG_FIFO                    0x00
#define REG_OP_MODE                 0x01
#define REG_FRF_MSB                 0x06
#define REG_FRF_MID                 0x07
#define REG_FRF_LSB                 0x08
#define REG_PA_CONFIG               0x09
#define REG_LNA                     0x0C
#define REG_FIFO_ADDR_PTR           0x0D
#define REG_FIFO_TX_BASE_ADDR       0x0E
#define REG_FIFO_RX_BASE_ADDR       0x0F
#define REG_FIFO_RX_CURRENT_ADDR    0x10
#define REG_IRQ_FLAGS               0x12
#define REG_RX_NB_BYTES             0x13
#define REG_PKT_SNR_VALUE           0x19
#define REG_PKT_RSSI_VALUE          0x1A
#define REG_MODEM_CONFIG_1          0x1D
#define REG_MODEM_CONFIG_2          0x1E
#define REG_PREAMBLE_MSB            0x20
#define REG_PREAMBLE_LSB            0x21
#define REG_PAYLOAD_LENGTH          0x22
#define REG_MODEM_CONFIG_3          0x26
#define REG_RSSI_WIDEBAND           0x2C
#define REG_DETECTION_OPTIMIZE      0x31
#define REG_DETECTION_THRESHOLD     0x37
#define REG_SYNC_WORD               0x39
#define REG_DIO_MAPPING_1           0x40
#define REG_VERSION                 0x42
#define REG_PA_DAC                  0x4D

/* Operation Modes */
#define MODE_LONG_RANGE_MODE        0x80
#define MODE_SLEEP                  0x00
#define MODE_STDBY                  0x01
#define MODE_TX                     0x03
#define MODE_RX_CONTINUOUS          0x05
#define MODE_RX_SINGLE              0x06

/* IRQ Flags */
#define IRQ_TX_DONE_MASK            0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK  0x20
#define IRQ_RX_DONE_MASK            0x40

/* PA Config */
#define PA_BOOST                    0x80

/* Max packet size */
#define MAX_PKT_LENGTH              255

/* LoRa Handle Structure */
typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *nss_port;
    uint16_t nss_pin;
    GPIO_TypeDef *rst_port;
    uint16_t rst_pin;
    GPIO_TypeDef *dio0_port;
    uint16_t dio0_pin;
    uint32_t frequency;
    uint8_t tx_power;
    uint8_t spreading_factor;
    uint32_t bandwidth;
    uint8_t coding_rate;
    uint8_t sync_word;
} SX127x_t;

/* Function Prototypes */
bool SX127x_Init(SX127x_t *lora);
uint8_t SX127x_ReadRegister(SX127x_t *lora, uint8_t reg);
void SX127x_WriteRegister(SX127x_t *lora, uint8_t reg, uint8_t value);
bool SX127x_Begin(SX127x_t *lora, uint32_t frequency);
void SX127x_Reset(SX127x_t *lora);
void SX127x_Sleep(SX127x_t *lora);
void SX127x_Idle(SX127x_t *lora);
bool SX127x_IsTransmitting(SX127x_t *lora);
void SX127x_SetFrequency(SX127x_t *lora, uint32_t frequency);
void SX127x_SetTxPower(SX127x_t *lora, uint8_t power);
void SX127x_SetSpreadingFactor(SX127x_t *lora, uint8_t sf);
void SX127x_SetBandwidth(SX127x_t *lora, uint32_t bw);
void SX127x_SetCodingRate(SX127x_t *lora, uint8_t cr);
void SX127x_SetSyncWord(SX127x_t *lora, uint8_t sw);
int SX127x_ParsePacket(SX127x_t *lora);
int SX127x_PacketRssi(SX127x_t *lora);
float SX127x_PacketSnr(SX127x_t *lora);
void SX127x_BeginPacket(SX127x_t *lora);
void SX127x_EndPacket(SX127x_t *lora);
void SX127x_Write(SX127x_t *lora, uint8_t *buf, size_t size);
void SX127x_WriteByte(SX127x_t *lora, uint8_t byte);
int SX127x_Available(SX127x_t *lora);
int SX127x_Read(SX127x_t *lora);
int SX127x_ReadBytes(SX127x_t *lora, uint8_t *buf, size_t size);
void SX127x_Receive(SX127x_t *lora);

#ifdef __cplusplus
}
#endif

#endif /* __SX127X_H */
