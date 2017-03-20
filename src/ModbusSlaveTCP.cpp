/**
 * Copyright (c) 2015, Yaacov Zamir <kobi.zamir@gmail.com>
 *
 * Permission to use, copy, modify, and/or distribute this software for any 
 * purpose with or without fee is hereby granted, provided that the above 
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES 
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF 
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR 
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES 
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN 
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF 
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF  THIS SOFTWARE.
 */

#include <inttypes.h>
#include <string.h>
#include "Arduino.h"
#include "Print.h"

#include <ESP8266WiFi.h>
#include "ModbusSlaveTCP.h"

#define MLEN 6

/**
 * Create a server that listen to port 502
 */
WiFiServer server(502);
WiFiClient client;

/**
 * Init the modbus object.
 *
 * @param unitID the modbus slave id.
 */
ModbusTCP::ModbusTCP(uint8_t _unitID) {
    // set modbus slave unit id
    unitID = _unitID;
}

/**
 * Begin server.
 */
void ModbusTCP::begin() {
    server.begin();
}

/**
 * wait for end of frame, parse request and answer it.
 */
int ModbusTCP::poll() {
    size_t lengthIn;
    size_t lengthOut;
    uint16_t crc;
    uint16_t address;
    uint16_t length;
    uint16_t status;
    uint8_t fc;
    
    /**
     * Check if there is new clients
     */
    if (server.hasClient()){
        // if client is free - connect
        if (!client || !client.connected()){
            if(client) client.stop();
            client = server.available();
            
        // client is not free - reject
        } else {
            WiFiClient serverClient = server.available();
            serverClient.stop();
        }
    }
    
    /**
     * Read one data frame:
     */
     
    // if we have data in buffer
    // read until end of data.
    if (client && client.connected() && client.available()) {
        lengthIn = 0;
        
        //get data from the telnet client and push it to the UART
        while(client.available() && lengthIn < MAX_BUFFER) {
            bufIn[lengthIn] = client.read();
            lengthIn++;
        }
    } else {
        return 0;
    }
    
    /**
     * Validate buffer.
     */
    // check minimum length.
    if (lengthIn < (MLEN + 6)) return 0;
    
    // check unit-id
    if (bufIn[MLEN + 0] != unitID) return 0;
    
    /**
     * Parse command
     */
    fc = bufIn[MLEN + 1];
    switch (fc) {
        case FC_READ_COILS: // read coils (digital out state)
        case FC_READ_DISCRETE_INPUT: // read input state (digital in)
            address = word(bufIn[MLEN + 2], bufIn[MLEN + 3]); // coil to set.
            length = word(bufIn[MLEN + 4], bufIn[MLEN + 5]);
            
            // sanity check.
            if (length > MAX_BUFFER) return 0;
            
            // check command length.
            if (lengthIn != (MLEN + 6)) return 0;
            
            // build valid empty answer.
            lengthOut = MLEN + 3 + (length - 1) / 8 + 1;
            bufOut[MLEN + 2] = (length - 1) / 8 + 1;
            
            // clear data out.
            memset(MLEN + bufOut + 3, 0, bufOut[2]);
            
            // if we have uset callback.
            if (cbVector[CB_READ_COILS]) {
                cbVector[CB_READ_COILS](fc, address, length);
            }
            break;
        case FC_READ_HOLDING_REGISTERS: // read holding registers (analog out state)
        case FC_READ_INPUT_REGISTERS: // read input registers (analog in)
            address = word(bufIn[MLEN + 2], bufIn[MLEN + 3]); // first register.
            length = word(bufIn[MLEN + 4], bufIn[MLEN + 5]); // number of registers to read.
            
            // sanity check.
            if (length > MAX_BUFFER) return 0;
            
            // check command length.
            if (lengthIn != (MLEN + 6)) return 0;
            
            // build valid empty answer.
            lengthOut = MLEN + 3 + 2 * length;
            bufOut[MLEN + 2] = 2 * length;
            
            // clear data out.
            memset(MLEN + bufOut + MLEN + 3, 0, bufOut[2]);
            
            // if we have uset callback.
            if (cbVector[CB_READ_REGISTERS]) {
                cbVector[CB_READ_REGISTERS](fc, address, length);
            }
            break;
        case FC_WRITE_COIL: // write one coil (digital out)
            address = word(bufIn[MLEN + 2], bufIn[MLEN + 3]); // coil to set
            status = word(bufIn[MLEN + 4], bufIn[MLEN + 5]); // 0xff00 - on, 0x0000 - off
            
            // check command length.
            if (lengthIn != (MLEN + 6)) return 0;
            
            // build valid empty answer.
            lengthOut = MLEN + 6;
            memcpy(MLEN + bufOut + 2, MLEN + bufIn + 2, 4);
            
            // if we have uset callback.
            if (cbVector[CB_WRITE_COIL]) {
                cbVector[CB_WRITE_COIL](fc, address, status == COIL_ON);
            }
            break;
        case FC_WRITE_MULTIPLE_REGISTERS: // write holding registers (analog out)
            address = word(bufIn[MLEN + 2], bufIn[MLEN + 3]); // first register
            length = word(bufIn[MLEN + 4], bufIn[MLEN + 5]); // number of registers to set
            
            // sanity check.
            if (length > MAX_BUFFER) return 0;
            
            // check command length
            if (lengthIn != (MLEN + 7 + length * 2)) return 0;
            
            // build valid empty answer.
            lengthOut = MLEN + 6;
            memcpy(MLEN + bufOut + 2, MLEN + bufIn + 2, 4);
            
            // if we have uset callback
            if (cbVector[CB_WRITE_MULTIPLE_REGISTERS]) {
                cbVector[CB_WRITE_MULTIPLE_REGISTERS](fc, address, length);
            }
            break;
        default:
            // unknown command
            return 0;
            break;
    }
    
    /**
     * Build answer
     */
    bufOut[0] = bufIn[0]; // transaction Identifier msb
    bufOut[1] = bufIn[1]; // transaction Identifier lsb
    bufOut[2] = 0; // protocol msb
    bufOut[3] = 0; // protocol lsb
    bufOut[4] = 0; // msg length msb
    bufOut[5] = lengthOut - MLEN; // msg length lsb
    
    bufOut[MLEN + 0] = unitID;
    bufOut[MLEN + 1] = fc;
    
    /**
     * Transmit
     */
    if (client && client.connected()) {
        client.write((uint8_t*)bufOut, lengthOut);
        delay(1);
    }
    
    return lengthOut;
}

/**
 * Read register value from input buffer.
 *
 * @param offset the register offset from first register in this buffer.
 * @return the reguster value from buffer.
 */
uint16_t ModbusTCP::readRegisterFromBuffer(int offset) {
    int address = MLEN + 7 + offset * 2;
    
    return word(bufIn[address], bufIn[address + 1]);
}

/**
 * Write coil state to output buffer.
 *
 * @param offset the coil offset from first coil in this buffer.
 * @param state the coil state to write into buffer (true / false)
 */
void ModbusTCP::writeCoilToBuffer(int offset, uint16_t state) {
    int address = MLEN + 3 + offset / 8;
    int bit = offset % 8;
    
    if (state == HIGH) {
        bitSet(bufOut[address], bit);
    } else if (!state) {
        bitClear(bufOut[address], bit);
    }
}

/**
 * Write register value to output buffer.
 *
 * @param offset the register offset from first register in this buffer.
 * @param value the register value to write into buffer.
 */
void ModbusTCP::writeRegisterToBuffer(int offset, uint16_t value) {
    int address = MLEN + 3 + offset * 2;
    
    bufOut[address] = value >> 8;
    bufOut[address + 1] = value & 0xff;
}

/**
 * Write arbitrary string of uint8_t to output buffer.
 *
 * @param offset the register offset from first register in this buffer.
 * @param str the string to write into buffer.
 * @param length the string length.
 */
void ModbusTCP::writeStringToBuffer(int offset, uint8_t *str, uint8_t length) {
    int address = MLEN + 3 + offset * 2;
    
    // check string length.
    if ((address + length) >= MAX_BUFFER) return;
    
    memcpy(bufOut + address, str, length);
}

