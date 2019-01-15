#ifndef VESCLIB_H_
#define VESCLIB_H_

#include "Arduino.h"
#include "crc.h"
#include "datatypes.h"
#include "buffer.h"

///SetSerialPort sets the serial to communicate with the VESC
///Multiple ports possible
void SetSerialPort(uint8_t _serialPort1, uint8_t _serialPort2, uint8_t _serialPort3, uint8_t _serialPort4);
void SetSerialPort(uint8_t _serialPort1, uint8_t _serialPort2, uint8_t _serialPort3);
void SetSerialPort(uint8_t _serialPort1, uint8_t _serialPort2);
void SetSerialPort(uint8_t _serialPort1);

static HardwareSerial* serialPort1;
static HardwareSerial* serialPort2;
static HardwareSerial* serialPort3;
static HardwareSerial* serialPort4;
static usb_serial_class* debugSerialPort = NULL;

void SetSerialPort(uint8_t _serialPort1, uint8_t  _serialPort2, uint8_t  _serialPort3, uint8_t  _serialPort4) {
	switch (_serialPort1) {
		case 1:
			serialPort1 = &Serial1;
			break;
		case 2:
			serialPort1 = &Serial2;
			break;
		case 3:
			serialPort1 = &Serial3;
			break;
		default:
			serialPort1 = &Serial1;
			break;
	}
	switch (_serialPort2) {
		case 1:
			serialPort2 = &Serial1;
			break;
		case 2:
			serialPort2 = &Serial2;
			break;
		case 3:
			serialPort2 = &Serial3;
			break;
		default:
			serialPort2 = &Serial1;
			break;
	}
	switch (_serialPort3) {
		case 1:
			serialPort3 = &Serial1;
			break;
		case 2:
			serialPort3 = &Serial2;
			break;
		case 3:
			serialPort3 = &Serial3;
			break;
		default:
			serialPort3 = &Serial1;
			break;
	}
}

void SetSerialPort(uint8_t _serialPort1) {
	SetSerialPort(_serialPort1, _serialPort1, _serialPort1, _serialPort1);
}
void SetSerialPort(uint8_t _serialPort1, uint8_t _serialPort2) {
	SetSerialPort(_serialPort1, _serialPort2, _serialPort1, _serialPort1);
}
void SetSerialPort(uint8_t _serialPort1,uint8_t _serialPort2,uint8_t _serialPort3) {
	SetSerialPort(_serialPort1, _serialPort2, _serialPort3, _serialPort1);
}

#include "ringbuffer.h"

enum possible_msg_states {
  OUT,
  START,
  TYPE,
  SIZE,
} msg_state[3];

int PackSendPayload(uint8_t* payload, int lenPay, int num) {
	uint16_t crcPayload = crc16(payload, lenPay);
	int count = 0;
	uint8_t messageSend[256];

	if (lenPay <= 256)
	{
		messageSend[count++] = 2;
		messageSend[count++] = lenPay;
	}
	else
	{
		messageSend[count++] = 3;
		messageSend[count++] = (uint8_t)(lenPay >> 8);
		messageSend[count++] = (uint8_t)(lenPay & 0xFF);
	}
	memcpy(&messageSend[count], payload, lenPay);

	count += lenPay;
	messageSend[count++] = (uint8_t)(crcPayload >> 8);
	messageSend[count++] = (uint8_t)(crcPayload & 0xFF);
	messageSend[count++] = 3;
	messageSend[count] = NULL;

	HardwareSerial *serial;

	switch (num) {
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
			break;
	}

	//Sending package
	serial->write(messageSend, count);


	//Returns number of send bytes
	return count;
}

void vesc_send_status_request(uint8_t serial_port) {
	uint8_t command[1] = { COMM_GET_VALUES };
	PackSendPayload(command, 1, serial_port);
}

void make_serial_available(uint8_t serial_port) {
    shift_rx_stream_to_buffer(serial_port);
}

uint8_t find_status_message(uint8_t serial_port) {
    static uint8_t msg_len = 0;
    while (Serial_available(serial_port)) {
        if      (msg_state[serial_port] == OUT)   msg_state[serial_port] = (Serial_read(serial_port) == 2) ? START : OUT; 
        else if (msg_state[serial_port] == START) msg_state[serial_port] = (Serial_peek(serial_port,1) == COMM_GET_VALUES) ? TYPE : OUT; 
        else if (msg_state[serial_port] == TYPE) {
            msg_len = Serial_peek(serial_port);
            msg_state[serial_port] = SIZE;
        }
        else if (msg_state[serial_port] == SIZE && Serial_available(serial_port) >= msg_len + 4) {
            uint16_t crc_comp = (((uint16_t)Serial_peek(serial_port,msg_len+1) << 8) & 0xFF00) + ((uint16_t)Serial_peek(serial_port,msg_len+2) & 0xFF);
            if (crc_comp == calculate_crc_from_ring_buffer(serial_port, 1,msg_len) && Serial_peek(serial_port,msg_len+3) == 3) {
                Serial_flush(serial_port, 1);
                return msg_len;
            }
            else {
                msg_state[serial_port] = OUT;
            }
        }
        else break;
    }
    return false;
}

bool vesc_compute_receive(uint8_t serial_port) {
    make_serial_available(serial_port);
    find_status_message(serial_port);
    return true;
}

bool VescUartGetValue(bldcMeasure& values, uint8_t serial_port) {
    make_serial_available(serial_port);
    uint8_t msg_len = find_status_message(serial_port);
	if (msg_len) {
		byte mess[70];
		int32_t ind = 0;
		transfer_data_from_ring_to_array(serial_port, mess, msg_len);
		uint8_t *message = mess;
		message++;
		values.tempFetFiltered		= buffer_get_float16(message, 1e1, &ind);
		values.tempMotorFiltered	= buffer_get_float16(message, 1e1, &ind);
		values.avgMotorCurrent		= buffer_get_float32(message, 100.0, &ind);
		values.avgInputCurrent		= buffer_get_float32(message, 100.0, &ind);
		values.avgId				= buffer_get_float32(message, 1e2, &ind);
		values.avgIq				= buffer_get_float32(message, 1e2, &ind);
		values.dutyNow				= buffer_get_float16(message, 1000.0, &ind);
		values.rpm					= buffer_get_float32(message, 1.0, &ind);
		values.inpVoltage			= buffer_get_float16(message, 10.0, &ind);
		values.ampHours				= buffer_get_float32(message, 10000.0, &ind);
		values.ampHoursCharged		= buffer_get_float32(message, 10000.0, &ind);
		values.wattHours			= buffer_get_float32(message, 1e4, &ind);
		values.watthoursCharged		= buffer_get_float32(message, 1e4, &ind);
		values.tachometer			= buffer_get_int32(message, &ind);
		values.tachometerAbs		= buffer_get_int32(message, &ind);
		values.faultCode			= message[ind];
	}
    return msg_len;
}

void VescUartSetCurrent(float current, int num) {
	int32_t index = 0;
	uint8_t payload[5];

	payload[index++] = COMM_SET_CURRENT ;
	buffer_append_int32(payload, (int32_t)(current * 1000), &index);
	PackSendPayload(payload, 5, num);
}

#endif // VESCLIB_H_