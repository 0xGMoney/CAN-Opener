#include "can_opener.h"

void CAN_OPENER_INIT(uint8_t can_baud_) {
	// Pull Slave Select pin (effectively disabling pin) HIGH.
	PORTB |= (1U<<SS);
	// Pull interrupt pin HIGH.
	PORTD |= (1U<<INTRPT);

	// Configure INPUTs and OUTPUTs.
	DDRB |= ((1U<<SCK) | (1U<<MOSI) | (1U<<SS));
	DDRB &= ~(1U<<MISO);
	DDRD &= ~(1U<<INTRPT);

	// Enable SPI interface.
	SPCR |= ((1U<<MSTR) | (1U<<SPE));
	SPCR &= ~(1U<<DORD);

	// Prescale external oscillator freq at the
	// ATmega328P by a factor of '4'. Will have
	// enough Time Quanta to run at 500kbps and
	// 250kbps.
	SPSR &= ~(1U<<SPI2X);
	SPCR &= ~((1U<<SPR1) | (1U<<SPR0));

	// Recommended reset to put MCP2515 into
	// Configuration mode.
	MCP_RESET();

	_delay_ms(100);

	// MCP2515 is now in Configuration mode.
	// Keep SS low so can write to multiple registers
	// at once (CNF3, CNF2, CNF1, CANINTE).
	TOGGLE_SS();
	SPI_WRITE(MCP_WRITE_CMD);
	SPI_WRITE(CNF3);
	if (can_baud_ == CAN_500KBPS) {
		SPI_WRITE(1U<<PHSEG21);
		SPI_WRITE((1U<<BLTMODE) | (1U<<PHSEG11));
	} else if (can_baud_ == CAN_250KBPS) {
		SPI_WRITE((1U<<PHSEG22) | (1U<<PHSEG20));
		SPI_WRITE((1U<<BLTMODE) | (1U<<PHSEG12) | (1U<<PHSEG11) | (1U<<PRSEG0));
	}
	SPI_WRITE(MCU_4MHZ_CLK);
	SPI_WRITE((1U<<RX1IE) | (1U<<RX0IE));
	TOGGLE_SS();

	// Configure RX buffer control registers, turn
	// masks/filters OFF.
	MCP_WRITE(RXB0CTRL, ((1U<<RXM1) | (1U<<RXM0)));
	MCP_WRITE(RXB1CTRL, ((1U<<RXM1) | (1U<<RXM0)));

	return;
}

void SPI_WRITE(uint8_t tx_data) {
	SPDR = tx_data;
	// Wait for SPIF to get set, indicating transfer complete.
	while (!(SPSR && (1U<<SPIF))) {
		;
	}

	return;
}

void MCP_RESET(void) {
	TOGGLE_SS();
	SPI_WRITE(MCP_RESET_CMD);
	TOGGLE_SS();

	return;
}

// TODO: How to read data back after sending command?
uint8_t MCP_READ(uint8_t register_) {
	TOGGLE_SS();
	SPI_WRITE(MCP_READ_CMD);
	SPI_WRITE(register_);
	SPI_WRITE(FILLER);
	TOGGLE_SS();

	return SPDR;
}

void MCP_READ_RX_BUFFER(uint8_t buffer_) {
	TOGGLE_SS();
	SPI_WRITE(MCP_READ_RX_BUFFER_CMD || buffer_);
	TOGGLE_SS();

	return;
}

void MCP_WRITE(uint8_t register_, uint8_t data_) {
	TOGGLE_SS();
	SPI_WRITE(MCP_WRITE_CMD);
	SPI_WRITE(register_);
	SPI_WRITE(data_);
	TOGGLE_SS();

	return;
}

void MCP_LOAD_TX_BUFFER(uint8_t buffer_) {
	TOGGLE_SS();
	SPI_WRITE(MCP_LOAD_TX_BUFFER_CMD || buffer_);
	TOGGLE_SS();

	return;
}

void MCP_RTS(uint8_t buffers_) {
	TOGGLE_SS();
	SPI_WRITE(MCP_RTS_CMD || buffers_);
	TOGGLE_SS();

	return;
}

void MCP_READ_STATUS(void) {
	TOGGLE_SS();
	SPI_WRITE(MCP_READ_STATUS_CMD);
	SPI_WRITE(FILLER);
	TOGGLE_SS();

	return;
}

uint8_t MCP_RX_STATUS(void) {
	TOGGLE_SS();
	SPI_WRITE(MCP_RX_STATUS_CMD);
	SPI_WRITE(FILLER);
	SPI_WRITE(FILLER);
	TOGGLE_SS();

	// 0xCO are the bits for RXB0 and RXB1 buffers
	return SPDR & 0xC0;
}

void MCP_BIT_MODIFY(uint8_t register_, uint8_t mask_, uint8_t data_) {
	TOGGLE_SS();
	SPI_WRITE(MCP_BIT_MODIFY_CMD);
	SPI_WRITE(register_);
	SPI_WRITE(mask_);
	SPI_WRITE(data_);
	TOGGLE_SS();

	return;
}

uint8_t SET_MCP_MODE(uint8_t requested_mode_) {
	MCP_WRITE(CANCTRL, (requested_mode_| (1U<<ABAT)));

	if ((MCP_READ(CANSTAT) & requested_mode_) != requested_mode_) {
		return 0;
	}

	return 1;
}

uint8_t MCP_RX_VALID_MSG(void) {
	const unsigned char rx_buffers = 0xC0;
	if (PORTD & (1U<<INTRPT)) {
		return (MCP_RX_STATUS() & rx_buffers);
	}

	return 0;
}

void MCP_READ_MSG(CAN_FRAME* frame) {
	uint8_t rx_buf_to_read = MCP_RX_VALID_MSG();
	const uint8_t rx_buf0 = 0x40;
	const uint8_t rx_buf1 = 0x80;
	const uint8_t RXB0 = 0x00;
	const uint8_t RXB1 = 0x04;
	const uint8_t is_extended_frame = 0x08;

	if (rx_buf_to_read & rx_buf0) {
		TOGGLE_SS();
		SPI_WRITE(MCP_READ_RX_BUFFER_CMD | RXB0);

		// Read Standard frame identifier.
		SPI_WRITE(FILLER);
		frame->frame_ident = (SPDR << 3);
		SPI_WRITE(FILLER);
		frame->frame_ident = (frame->frame_ident | (SPDR >> 5));

		// If an Extended frame, read Extended frame identifier.
		if (SPDR & is_extended_frame) {
			SPI_WRITE(FILLER);
			frame->frame_ex_ident = (SPDR << 8);
			SPI_WRITE(FILLER);
			frame->frame_ex_ident = frame->frame_ex_ident & SPDR;
		} else {
			SPI_WRITE(FILLER);
			SPI_WRITE(FILLER);
		}

		// Read number of data 'words' and write them to array.
		SPI_WRITE(FILLER);
		frame->frame_dlc = (SPDR & 0x0F);
		for (int ii = 0; ii < frame->frame_dlc; ii++) {
       			SPI_WRITE(FILLER);
			frame->frame_data[ii] = SPDR;
		}
		TOGGLE_SS();

		// Clear interrupt flag.
		MCP_BIT_MODIFY(CANINTF, 0x01, 0x00);
	}
}
