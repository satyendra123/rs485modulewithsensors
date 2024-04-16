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


//EXAMPLE-2 this is the SHT20 sensor which gives the data through rs485tottl. so we will see both the ways here one is using the software serial. and the other one is using the modbus
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
