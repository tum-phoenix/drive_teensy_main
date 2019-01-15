#ifndef RINGBUFFER_H_
#define RINGBUFFER_H_

#include "Arduino.h"
#include "crc.h"


#define BUFFER_LEN 255                               // length of Serial buffer (recomended to be far bigger than max msg length)
#define RING_SIZE 256                                // size of TX_buffer and RX_buffer
#define NUM_UART 3
unsigned char RX_ring[NUM_UART][RING_SIZE];
uint8_t TX_head[NUM_UART];
uint8_t RX_head[NUM_UART];
uint8_t RX_tail[NUM_UART];

uint8_t Serial_available(uint8_t port) {
  // returns the number of available bytes from the ringbuffer.
  if (RX_head[port] >= RX_tail[port]) {
    return (RX_head[port] - RX_tail[port]);
  }
  else {
    return ((RING_SIZE - RX_tail[port]) + RX_head[port]);
  }
}

byte Serial_read(uint8_t port) {
  // reads the next byte from the readbuffer and increases the curser
  // position automatically
  byte temp = RX_ring[port][RX_tail[port]];
  if (++RX_tail[port] == RING_SIZE) RX_tail[port] = 0;
  return temp;
}

byte Serial_peek(uint8_t port, uint8_t steps) {
  // reads a byte from the available bytes without changing the curser position
  // basicalle: sneak into (available) future.
    if (steps < Serial_available(port)) {
        if ((RX_tail[port] + steps) < RING_SIZE) {
            return RX_ring[port][RX_tail[port]+steps];
        }
        else {
            return RX_ring[port][RX_tail[port] + steps - RING_SIZE];
        }
    }
    else return 0;
}

byte Serial_peek(uint8_t port) {
  // reads the next byte without increasing the courser position
  // same as read but without curser movement
  return Serial_peek(port, 0);
}

void shift_rx_stream_to_buffer(uint8_t port) {
  // read from serial port into ringbuffer.
  HardwareSerial *serial;

	switch (port) {
		case 0:
			serial=serialPort1;
			break;
		case 1:
			serial=serialPort2;
			break;
		case 2:
			serial=serialPort3;
			break;
		case 3:
			serial=serialPort4;
			break;
		default:
      return;     // in case no valid port is selected do nothing
			break;
	}

  while(serial->available()) {
      RX_ring[port][RX_head[port]] = serial->read();
      if (++RX_head[port] == RING_SIZE) RX_head[port] = 0;
  }
}

uint16_t calculate_crc_from_ring_buffer(uint8_t port, uint8_t start_i, uint8_t len) {
  static uint8_t payload[251];
  for (uint8_t i = 0; i < len; i++) {
       payload[i] = Serial_peek(port,start_i+i);
  }
  return crc16(payload, len);
}

void Serial_flush(uint8_t port, uint8_t num) {
  // reads a fixed number of bytes from the buffer and discardes them
  for (uint8_t i = 0; i < num; i++) {
      Serial_read(port);
  }
}
void transfer_data_from_ring_to_array(uint8_t port, byte* buffer, uint8_t len) {
  // reads data from the ringbuffer into a byte array while increasing the
  // curser position
  for (uint8_t i = 0; i < len; i++) {
      buffer[i] = Serial_read(port);
  }
}

#endif // RINGBUFFER_H
