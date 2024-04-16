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

//EXAMPLE-2 this is the SHT20 sensor which gives the data through rs485tottl. so we will see both the ways here one is using the software serial. and the other one is using the modbus
