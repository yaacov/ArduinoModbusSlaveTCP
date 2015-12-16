/*
    Modbus slave simple example

    Control and Read Arduino I/Os using Modbus serial connection.
    
    This sketch show how to use the callback vector for reading and
    controleing Arduino I/Os.
    
    * Controls digital output pins as modbus coils.
    * Reads digital inputs state as discreet inputs.
    * Reads analog inputs as input registers.

    The circuit: ( see: ./extras/ModbusSetch.pdf )
    * An ESP8266 unit with connectors to digital out pins.
    * 2 x LEDs, with 220 ohm resistors in series.
    * A switch connected to a digital input pin.
    * A potentiometer connected to an analog input pin.

    Created 8 12 2015
    By Yaacov Zamir

    https://github.com/yaacov/ArduinoModbusSlave

*/

#include <ESP8266WiFi.h>
#include <ModbusSlaveTCP.h>

const char* ssid = "home";
const char* pass = "1234567890abc";

/* slave id = 1, rs485 control-pin = 8, baud = 9600
 */
#define SLAVE_ID 1

/**
 *  Modbus object declaration
 */
ModbusTCP slave(SLAVE_ID);

void setup() {
    /* Start serial port
     */
    Serial.begin(115200);
    
    /* Connect WiFi to the network
     */
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, pass);
    
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    
    /* register handler functions
     * into the modbus slave callback vector.
     */
    slave.cbVector[CB_WRITE_COIL] = writeDigitlOut;
    slave.cbVector[CB_READ_COILS] = readDigitalIn;
    slave.cbVector[CB_READ_REGISTERS] = readAnalogIn;
    
    /* start slave and listen to TCP port 502
     */
    slave.begin();
    
    // log to serial port
    Serial.println("");
    Serial.print("Modbus ready, listen on ");
    Serial.print(WiFi.localIP());
    Serial.println(" : 502");
}

void loop() {
    /* listen for modbus commands con serial port
     *
     * on a request, handle the request.
     * if the request has a user handler function registered in cbVector
     * call the user handler function.
     */ 
    slave.poll();
}

/**
 * Handel Force Single Coil (FC=05)
 * set digital output pins (coils) on and off
 */
void writeDigitlOut(uint8_t fc, uint16_t address, uint16_t status) {
    digitalWrite(address, status);
}

/**
 * Handel Read Input Status (FC=02/01)
 * write back the values from digital in pins (input status).
 *
 * handler functions must return void and take:
 *      uint8_t  fc - function code
 *      uint16_t address - first register/coil address
 *      uint16_t length/status - length of data / coil status
 */
void readDigitalIn(uint8_t fc, uint16_t address, uint16_t length) {
    // read digital input
    for (int i = 0; i < length; i++) {
        slave.writeCoilToBuffer(i, digitalRead(address + i));
    }
}

/**
 * Handel Read Input Registers (FC=04/03)
 * write back the values from analog in pins (input registers).
 */
void readAnalogIn(uint8_t fc, uint16_t address, uint16_t length) {
    // read analog input
    for (int i = 0; i < length; i++) {
        slave.writeRegisterToBuffer(i, analogRead(address + i));
    }
}

