/**
  ******************************************************************************
  * @file           : lora_protocol.h
  * @brief          : LoRa framed protocol header (compatible with Arduino)
  ******************************************************************************
  */

#ifndef __LORA_PROTOCOL_H
#define __LORA_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "sx127x.h"
#include <stdint.h>
#include <stdbool.h>

/* Protocol Addresses */
#define ADDR_ARDUINO  0x02
#define ADDR_STM32    0x01

/* Packet Types */
#define PKT_TYPE_DATA 0x01
#define PKT_TYPE_ACK  0x02

/* Protocol Constants */
#define MAX_PAYLOAD_SIZE 250

/* Packet Structure */
typedef struct {
    uint8_t dst;      // Destination address
    uint8_t src;      // Source address
    uint8_t type;     // Packet type (DATA or ACK)
    uint8_t seq;      // Sequence number
    uint8_t len;      // Payload length
    uint8_t payload[MAX_PAYLOAD_SIZE];
} LoRa_Packet_t;

/* Protocol Handle */
typedef struct {
    SX127x_t *lora;
    uint8_t seq_counter;
    void (*data_received_callback)(char *msg, uint8_t len);
    void (*ack_received_callback)(uint8_t seq);
} LoRa_Protocol_t;

/* Function Prototypes */
void LoRa_Protocol_Init(LoRa_Protocol_t *proto, SX127x_t *lora);
bool LoRa_Protocol_SendData(LoRa_Protocol_t *proto, uint8_t dst, const char *msg);
bool LoRa_Protocol_SendAck(LoRa_Protocol_t *proto, uint8_t dst, uint8_t seq);
void LoRa_Protocol_ProcessIncoming(LoRa_Protocol_t *proto);

#ifdef __cplusplus
}
#endif

#endif /* __LORA_PROTOCOL_H */
