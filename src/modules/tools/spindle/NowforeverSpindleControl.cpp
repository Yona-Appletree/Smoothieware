/*
      This file is part of Smoothie (http://smoothieware.org/). The motion control part is heavily based on Grbl (https://github.com/simen/grbl).
      Smoothie is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
      Smoothie is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
      You should have received a copy of the GNU General Public License along with Smoothie. If not, see <http://www.gnu.org/licenses/>.
*/

/* 
    Nowforever RS485 communication protocol

    Adapted from the Nowforever E100 manual, downloaded from http://www.c-n-c.cz/download/file.php?id=31939

    This module makes the assumption that the Nowforever VFD is configured for address 1.

    == Nowforever E100 Register Map ==

    Adapted from page 104 of the manual.

    +-----------------+----------------------------------------------------------+
    | Modbus Register | Register Info                                            |
    +-----------------+----------------------------------------------------------+
    |           0x900 | Bit 0: Spindle ON/OFF (1/0)                              |
    |                 | Bit 1: Spindle ON/OFF (1/0)                              |
    |                 | Bit 2: Spindle ON/OFF (1/0)                              |
    |                 | Bit 3: Spindle ON/OFF (1/0)                              |
    |                 | Bit 4-F: Reserved                                        |
    |           0x901 | Spindle Frequency                                        |
    |           0x902 | PID Settings                                             |
    |           0x909 | Save To EEPROM by writing 0x01                           |
    +-----------------+----------------------------------------------------------+
*/

#include "libs/Kernel.h"
#include "StreamOutputPool.h"
#include "BufferedSoftSerial.h"
#include "ModbusSpindleControl.h"
#include "NowforeverSpindleControl.h"
#include "gpio.h"
#include "Modbus.h"

void NowforeverSpindleControl::turn_on() 
{
    //THEKERNEL->streams->printf("Spindle ON START\n");

    // prepare data for the spindle off command
    char turn_on_msg[] = {
        0x01, // Address
        0x10, // Write Multiple Registers
        0x09, // Start Address (MSB)
        0x00, // Start Address (LSB)
        0x00, // Register Count (MSB)
        0x01, // Register Count (LSB)
        0x02, // Byte Count
        0x00, // Data 0 (MSB)
        0b00000001, // Data 0 (LSB)
        0, // CRC LSB
        0  // CRC MSB
    };
    // calculate CRC16 checksum
    unsigned int crc = modbus->crc16(turn_on_msg, sizeof(turn_on_msg)-2);
    turn_on_msg[sizeof(turn_on_msg) - 2] = crc & 0xFF;
    turn_on_msg[sizeof(turn_on_msg) - 1] = (crc >> 8);

    // enable transmitter
    modbus->dir_output->set();
    modbus->delay(1);
    // send the actual message
    modbus->serial->write(turn_on_msg, sizeof(turn_on_msg));
    // wait a calculated time for the data to be sent
    modbus->delay((int) ceil(sizeof(turn_on_msg) * modbus->delay_time));
    // disable transmitter
    modbus->dir_output->clear();
    // wait 50ms, required by the Modbus standard
    modbus->delay(50);
    spindle_on = true;

    //THEKERNEL->streams->printf("Spindle ON END\n");
}

void NowforeverSpindleControl::turn_off() 
{
    //THEKERNEL->streams->printf("Spindle OFF START\n");

    // prepare data for the spindle off command
    char turn_on_msg[] = {
        0x01, // Address
        0x10, // Write Multiple Registers
        0x09, // Start Address (MSB)
        0x00, // Start Address (LSB)
        0x00, // Register Count (MSB)
        0x01, // Register Count (LSB)
        0x02, // Byte Count
        0x00, // Data 0 (MSB)
        0b00000000, // Data 0 (LSB)
        0, // CRC LSB
        0  // CRC MSB
    };
    // calculate CRC16 checksum
    unsigned int crc = modbus->crc16(turn_on_msg, sizeof(turn_on_msg)-2);
    turn_on_msg[sizeof(turn_on_msg) - 2] = crc & 0xFF;
    turn_on_msg[sizeof(turn_on_msg) - 1] = (crc >> 8);

    // enable transmitter
    modbus->dir_output->set();
    modbus->delay(1);
    // send the actual message
    modbus->serial->write(turn_on_msg, sizeof(turn_on_msg));
    // wait a calculated time for the data to be sent
    modbus->delay((int) ceil(sizeof(turn_on_msg) * modbus->delay_time));
    // disable transmitter
    modbus->dir_output->clear();
    // wait 50ms, required by the Modbus standard
    modbus->delay(50);
    spindle_on = false;

    //THEKERNEL->streams->printf("Spindle OFF END\n");
}

void NowforeverSpindleControl::set_speed(int target_rpm) 
{
    //THEKERNEL->streams->printf("Spindle Set Speed %d START\n", target_rpm);
    uint16_t hz100 = target_rpm / 60 * 100;

    // prepare data for the spindle off command
    char turn_on_msg[] = {
        0x01, // Address
        0x10, // Write Multiple Registers
        0x09, // Start Address (MSB)
        0x01, // Start Address (LSB)
        0x00, // Register Count (MSB)
        0x01, // Register Count (LSB)
        0x02, // Byte Count
        hz100 >> 8, // Data 0 (MSB)
        hz100 & 0xFF, // Data 0 (LSB)
        0, // CRC LSB
        0  // CRC MSB
    };
    // calculate CRC16 checksum
    unsigned int crc = modbus->crc16(turn_on_msg, sizeof(turn_on_msg)-2);
    turn_on_msg[sizeof(turn_on_msg) - 2] = crc & 0xFF;
    turn_on_msg[sizeof(turn_on_msg) - 1] = (crc >> 8);

    // enable transmitter
    modbus->dir_output->set();
    modbus->delay(1);
    // send the actual message
    modbus->serial->write(turn_on_msg, sizeof(turn_on_msg));
    // wait a calculated time for the data to be sent
    modbus->delay((int) ceil(sizeof(turn_on_msg) * modbus->delay_time));
    // disable transmitter
    modbus->dir_output->clear();
    // wait 50ms, required by the Modbus standard
    modbus->delay(50);

    //THEKERNEL->streams->printf("Spindle Set Speed %d END\n", target_rpm);
}

void NowforeverSpindleControl::report_speed() 
{
    // clear RX buffer before start
    while(modbus->serial->readable()){
        modbus->serial->getc();
    }

    // prepare data for the get speed command
    char get_speed_msg[] = {
        0x01, // Address
        0x03, // Read Register
        0x05, // Start Address (MSB)
        0x02, // Start Address (LSB)
        0x00, // Number (MSB)
        0x01, // Number (LSB)
        0x00, // CRC LSB
        0x00, // CRC MSB
    };
    // calculate CRC16 checksum
    unsigned int crc = modbus->crc16(get_speed_msg, sizeof(get_speed_msg)-2);
    get_speed_msg[sizeof(get_speed_msg) - 2] = crc & 0xFF;
    get_speed_msg[sizeof(get_speed_msg) - 1] = (crc >> 8);

    // enable transmitter
    modbus->dir_output->set();
    modbus->delay(1);
    // send the actual message
    modbus->serial->write(get_speed_msg, sizeof(get_speed_msg));
    // wait a calculated time for the data to be sent 
    modbus->delay((int) ceil(sizeof(get_speed_msg) * modbus->delay_time));
    // disable transmitter
    modbus->dir_output->clear();
    // wait 50ms, required by the Modbus standard
    modbus->delay(50);

    // wait for the complete message to be received
    modbus->delay((int) ceil(8 * modbus->delay_time));
    // prepare an array for the answer
    char speed[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

    // read the answer into the buffer
    for(int i=0; i<8; i++) {
        speed[i] = modbus->serial->getc();
    }
    // get the Hz value from trhe answer and convert it into an RPM value
    unsigned int hz = (speed[3] << 8) | speed[4];
    unsigned int rpm = hz / 100 * 60;

    // report the current RPM value
    THEKERNEL->streams->printf("Current RPM: %d\n", rpm);
}
