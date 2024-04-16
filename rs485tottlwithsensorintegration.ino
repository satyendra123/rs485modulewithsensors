//EXAMPLE-1 this is our first sensor co2 sensor which gives the data through rs485. so we read the sensor data and print on the serial monitor. so we actually understand the code. so we have 9 byte request data
// and we get the 9 byte response data from the sensor. now we use two serial monitor one is for serialwrite so that we can send the request data to sensor. and the another one is for normal serial.begin(9600) which will
capture the response and we can see the res[ponse data.

#include <SoftwareSerial.h>; // Include the SoftwareSerial library
SoftwareSerial mySerial(A0, A1); // A0 - connects to sensor's TX, A1 - connects to sensor's RX
byte cmd[9] = {0xFF,0x01,0x86,0x00,0x00,0x00,0x00,0x00,0x79}; // Command to request data
unsigned char response[9]; // Array to store sensor response

void setup() {
  Serial.begin(9600); // Start serial communication with the computer
  mySerial.begin(9600); // Start Software Serial communication with the sensor
}

void loop() {
  mySerial.write(cmd, 9); // Send the command to the sensor
  memset(response, 0, 9); // Clear the response array
  mySerial.readBytes(response, 9); // Read sensor response

  // Calculate checksum
  int i;
  byte crc = 0;
  for (i = 1; i < 8; i++) {
    crc += response[i];
  }
  crc = 255 - crc;
  crc++;

  // Check if the response is valid
  if (!(response[0] == 0xFF && response[1] == 0x86 && response[8] == crc)) {
    Serial.println("CRC error: " + String(crc) + " / "+ String(response[8])); // Print CRC error if found
  } else {
    // Calculate CO2 concentration
    unsigned int responseHigh = (unsigned int) response[2];
    unsigned int responseLow = (unsigned int) response[3];
    unsigned int ppm = (256 * responseHigh) + responseLow;
    Serial.println(ppm); // Print CO2 concentration
  }
  delay(10000); // Wait for 10 seconds before next reading
}
NOTE- maine is data to docklight software me bhi read kiya hai. aur mujhe waha par data mil rha hai. hex me. so maine ek prototype ka code likha apne python code ko hi sensor bana diya. aur bol diya ki agar ye 
request mile to usko ye response send kar dena. aur data mujhe apne arduino me show ho rha tha actual me.
import serial
import time
import random

serial_port = 'COM6'
baud_rate = 9600

ser = serial.Serial(serial_port, baud_rate, timeout=1)

def generate_response():
    ppm = random.randint(0, 2000)
    response = [0xFF, 0x86, ppm // 256, ppm % 256]
    # Calculate checksum
    crc = 255 - sum(response[1:]) % 256
    response.append(crc)
    return response

def is_valid_request(data):
    return len(data) == 9 and data[0] == 0xFF

# Main loop
while True:
    data = ser.read(9)
    if data and is_valid_request(data):
        print("Received request:", data)
        response = generate_response()
        crc = 255 - sum(response[1:]) % 256
        response[-1] = crc
        print("Sending response:", response)
        ser.write(bytearray(response))
        ser.flush()
    time.sleep(0.1)


//EXAMPLE-2 this is the TDR sensor which gives the data through rs485tottl. so we will see both the ways here one is using the software serial. and the other one is using the modbus
#include <SoftwareSerial.h>
#include <Wire.h>

#define RE 8
#define DE 7

const byte hum_temp_ec[8] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x03, 0x05, 0xCB};
byte sensorResponse[12] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte sensor_values[11];

SoftwareSerial mod(2, 3);

void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);
    mod.begin(4800);
    pinMode(RE, OUTPUT);
    pinMode(DE, OUTPUT);
    digitalWrite(RE, LOW);
    digitalWrite(DE, LOW);
    delay(100);
}

void loop() {
    /************** Soil EC Reading *******************/
    digitalWrite(DE, HIGH);
    digitalWrite(RE, HIGH);
    memset(sensor_values, 0, sizeof(sensor_values));
    delay(100);
    if (mod.write(hum_temp_ec, sizeof(hum_temp_ec)) == 8) {
        digitalWrite(DE, LOW);
        digitalWrite(RE, LOW);
        for (byte i = 0; i < 12; i++) {
            //Serial.print(mod.read(),HEX);
            sensorResponse[i] = mod.read();
            yield();
            Serial.print(sensorResponse[i], HEX);
            Serial.print(",");
        }
        Serial.println();
    }

    delay(250);

    // get sensor response data
    float soil_hum = 0.1 * int(sensorResponse[3] << 8 | sensorResponse[4]);
    float soil_temp = 0.1 * int(sensorResponse[5] << 8 | sensorResponse[6]);
    int soil_ec = int(sensorResponse[7] << 8 | sensorResponse[8]);
    soil_ec = 1.93 * soil_ec - 270.8;
    soil_ec = soil_ec / (1.0 + 0.019 * (soil_temp - 25));
    float soil_apparent_dieletric_constant = 1.3088 + 0.1439 * soil_hum + 0.0076 * soil_hum * soil_hum;
    float soil_bulk_permittivity = soil_apparent_dieletric_constant;  /// Hamed 2015 (apparent_dieletric_constant is the real part of permittivity)
    float soil_pore_permittivity = 80.3 - 0.37 * (soil_temp - 20); /// same as water 80.3 and corrected for temperature
    // converting bulk EC to pore water EC
    float soil_pw_ec;
    if (soil_bulk_permittivity > 4.1)
        soil_pw_ec = ((soil_pore_permittivity * soil_ec) / (soil_bulk_permittivity - 4.1) / 1000); /// from Hilhorst 2000.
    else
        soil_pw_ec = 0;

    Serial.print("Humidity: ");
    Serial.print(soil_hum);
    Serial.println(" %");
    Serial.print("Temperature: ");
    Serial.print(soil_temp);
    Serial.println(" °C");
    Serial.print("EC: ");
    Serial.print(soil_ec);
    Serial.println(" us/cm");
    Serial.print("pwEC: ");
    Serial.print(soil_pw_ec);
    Serial.println(" dS/m");
    Serial.print("soil_bulk_permittivity: ");
    Serial.println(soil_bulk_permittivity);
    delay(2000);
}

// Example-3 this is my SHT20 temperature and humidity sensor which gives the data through rs485 port. so it also support the modbus. so we will see both the ways to read the sensor data.
/*
  This code demonstrates how to interact with an Arduino Mega 2560 and
  a Modbus RTU temperature and humidity sensor (SHT20). It reads the
  temperature and humidity values every 1 seconds and display data to
  the serial monitor.

  Note: Serial Port 0 is not used to connect the RS485 Converter (MAX485)
  because its used for debugging. The Serial Port 1 (TX1, RX1) is used
  for ModBus communication interface.

  Wiring of Sensor, Arduino, and MAX485 TTL to RS485 Converter:
  ___________________________________________________________________________________________
  | Sensor (SHT20)   |   MAX485 TTL to RS485 Converter
  |  A (Yellow)      |        A (Terminal block)
  |  B (White)       |        B (Terminal block)
  |  GND (Black)     |       GND (External Supply)
  |  Vs (Red)        |      9-30V (External Supply)
  ___________________________________________________________________________________________
  | MAX485 TTL to RS485 Converter  |  Arduino (Hardware Serial)  |  Arduino (Software Serial)
  |     RO (Reciever Output)       |        D19 (RX1)            |          D9 (RX)
  |     RE (Reciever Enable)       |        D2                   |          D2
  |     DE (Driver Enable)         |        D3                   |          D3
  |     DI (Driver Input)          |        D18 (TX1)            |          D10 (TX)
  ___________________________________________________________________________________________
*/

#include <ModbusMaster.h>

#define MAX485_RE_NEG  2
#define MAX485_DE      3

ModbusMaster node;

void preTransmission() {
  digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1);
}

void postTransmission() {
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
}

void setup() {
  // Initialize control pins
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);

  // Modbus communication runs at 9600 baud
  Serial.begin(9600);
  Serial1.begin(9600);

  // Modbus slave ID 1
  node.begin(1, Serial1);

  // Callbacks allow us to configure the RS485 transceiver correctly
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
}

void loop() {
  // Request 2 registers starting at 0x0001
  uint8_t result = node.readInputRegisters(0x0001, 2);
  Serial.println("Data Requested");

  if (result == node.ku8MBSuccess) {
    // Get response data from sensor
    Serial.print("Temperature: ");
    Serial.print(float(node.getResponseBuffer(0) / 10.00F));
    Serial.print("   Humidity: ");
    Serial.println(float(node.getResponseBuffer(1) / 10.00F));
  }
  delay(1000);
}

Note- same isi chiz ko hum dekhte hai ki mai softwareserial ke sath kaise karunga.

/*
  This code demonstrates how to interact with an Arduino Mega 2560 and
  a Modbus RTU temperature and humidity sensor (SHT20). It reads the
  temperature and humidity values every 1 seconds and display data to
  the serial monitor.

  Note: Serial Port 0 is not used to connect the RS485 Converter (MAX485)
  because its used for debugging. The Serial Port 1 (TX1, RX1) is used
  for ModBus communication interface.

  Wiring of Sensor, Arduino, and MAX485 TTL to RS485 Converter:
  ___________________________________________________________________________________________
  | Sensor (SHT20)   |   MAX485 TTL to RS485 Converter
  |  A (Yellow)      |        A (Terminal block)
  |  B (White)       |        B (Terminal block)
  |  GND (Black)     |       GND (External Supply)
  |  Vs (Red)        |      9-30V (External Supply)
  ___________________________________________________________________________________________
  | MAX485 TTL to RS485 Converter  |  Arduino (Hardware Serial)  |  Arduino (Software Serial)
  |     RO (Reciever Output)       |        D19 (RX1)            |          D9 (RX)
  |     RE (Reciever Enable)       |        D2                   |          D2
  |     DE (Driver Enable)         |        D3                   |          D3
  |     DI (Driver Input)          |        D18 (TX1)            |          D10 (TX)
  ___________________________________________________________________________________________
*/

#include <ModbusMaster.h>
#include <SoftwareSerial.h>

#define MAX485_RE_NEG  2
#define MAX485_DE      3
#define SSERIAL_RX_PIN 10
#define SSERIAL_TX_PIN 11

SoftwareSerial RS485Serial(SSERIAL_RX_PIN, SSERIAL_TX_PIN);
ModbusMaster node;

void preTransmission() {
  digitalWrite(MAX485_RE_NEG, 1);
  digitalWrite(MAX485_DE, 1);
}

void postTransmission() {
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);
}

void setup() {
  // Initialize control pins
  pinMode(MAX485_RE_NEG, OUTPUT);
  pinMode(MAX485_DE, OUTPUT);
  digitalWrite(MAX485_RE_NEG, 0);
  digitalWrite(MAX485_DE, 0);

  // Modbus communication runs at 9600 baud
  Serial.begin(9600);
  RS485Serial.begin(9600);

  // Modbus slave ID 1
  node.begin(1, RS485Serial);

  // Callbacks allow us to configure the RS485 transceiver correctly
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
}

void loop() {
  // Request 2 registers starting at 0x0001
  uint8_t result = node.readInputRegisters(0x0001, 2);
  Serial.println("Data Requested");

  if (result == node.ku8MBSuccess) {
    // Get response data from sensor
    Serial.print("Temperature: ");
    Serial.print(float(node.getResponseBuffer(0) / 10.00F));
    Serial.print("   Humidity: ");
    Serial.println(float(node.getResponseBuffer(1) / 10.00F));
  }
  delay(1000);
}
//EXAMPLE-4 ab hum finally dekhte hai ki bina modbus ka use kiye hum kaise data ko read kar sakte hai
#include <SoftwareSerial.h>

SoftwareSerial sht20(8, 9);

byte tempRequest[8] = {0x01, 0x04, 0x00, 0x01, 0x00, 0x01, 0x60, 0x0a};
int lastRequest = 0;
char receivedByte;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while(!Serial) {
    ;
  }

  sht20.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (millis() - lastRequest > 10000) {
    for (int i = 0; i < 8; i++) {
      sht20.write(tempRequest[i]);
    }
    lastRequest = millis();
  }

  while (sht20.available()) {
    receivedByte = sht20.read();
    Serial.print(receivedByte, HEX);
  }
  Serial.println();
}

// Compute the MODBUS RTU CRC
uint16_t ModRTU_CRC(byte * buf, int len)
{
  uint16_t crc = 0xFFFF;
  
  for (int pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)buf[pos];          // XOR byte into least sig. byte of crc
  
    for (int i = 8; i != 0; i--) {    // Loop over each bit
      if ((crc & 0x0001) != 0) {      // If the LSB is set
        crc >>= 1;                    // Shift right and XOR 0xA001
        crc ^= 0xA001;
      }
      else                            // Else LSB is not set
        crc >>= 1;                    // Just shift right
    }
  }
  // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
  return crc;  
}
