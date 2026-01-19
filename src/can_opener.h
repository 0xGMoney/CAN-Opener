#ifndef CAN_OPENER_H
#define CAN_OPENER_H

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include "mcp2515_registers.h"

typedef struct {
	uint16_t frame_ident;
	uint16_t frame_ex_ident;
	uint8_t frame_dlc;
	uint8_t frame_data[8];
} CAN_FRAME;

#define CAN_500KBPS 0
#define CAN_250KBPS 1

#define MOSI   PORTB3
#define MISO   PORTB4
#define SS     PORTB2
#define SCK    PORTB5
#define INTRPT PORTD2

#define NORMAL_MODE 0x00U
#define SLEEP_MODE  0x20U
#define LPBK_MODE   0x40U
#define LISTEN_MODE 0x60U
#define CONFIG_MODE 0x80U

#define MCP_RESET_CMD           0xC0U
#define MCP_READ_CMD            0x03U
#define MCP_READ_RX_BUFFER_CMD  0x90U
#define MCP_WRITE_CMD           0x02U
#define MCP_LOAD_TX_BUFFER_CMD  0x40U
#define MCP_RTS_CMD             0x80U
#define MCP_READ_STATUS_CMD     0xA0U
#define MCP_RX_STATUS_CMD       0xB0U
#define MCP_BIT_MODIFY_CMD      0x05U

#define TOGGLE_SS() (PORTB ^= (1U<<SS))

#define FILLER 0xFF

void CAN_OPENER_INIT(uint8_t);

void SPI_WRITE(uint8_t);

void MCP_RESET(void);

void MCP_READ_RX_BUFFER(uint8_t);

uint8_t MCP_READ(uint8_t);

void MCP_WRITE(uint8_t, uint8_t);

void MCP_LOAD_TX_BUFFER(uint8_t);

void MCP_RTS(uint8_t);

void MCP_READ_STATUS(void);

uint8_t MCP_RX_STATUS(void);

void MCP_BIT_MODIFY(uint8_t, uint8_t, uint8_t);

uint8_t SET_MCP_MODE(uint8_t);

uint8_t MCP_RX_VALID_MSG(void);

void MCP_READ_MSG(CAN_FRAME*);

#endif	// CAN_OPENER_H
