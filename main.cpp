// Awnsers Modbus requests with fakeValue.
// TODO: Get temperaturedata from bluetoothsensors to replace fakeValue

#include <Arduino.h>

#define MAX_BUFFER 64
#define DE_PIN 26

int count = 0;
boolean response = 0;
int aResponse = 0;
uint16_t last_receive_len;
uint32_t last_receive_time;
uint32_t timeout;
uint8_t bufIn[MAX_BUFFER];
uint8_t bufOut[MAX_BUFFER];
int unitID = 1;
int fakeValue = 250;

void setup() {
    pinMode(DE_PIN, OUTPUT);
    digitalWrite(DE_PIN, LOW);
    Serial.begin( 115200 );
    Serial2.begin(9600, SERIAL_8E1);
    timeout = 35000000 / 9600;
}

unsigned int mWord(unsigned char h, unsigned char l) { return (h << 8) | l; }

uint16_t calcCRC(uint8_t *buf, int length) {
  int i, j;
  uint16_t crc = 0xFFFF;
  uint16_t tmp;

  // calculate crc16
  for (i = 0; i < length; i++) {
    crc = crc ^ buf[i];

    for (j = 0; j < 8; j++) {
      tmp = crc & 0x0001;
      crc = crc >> 1;
      if (tmp) {
        crc = crc ^ 0xA001;
      }
    }
  }
  return crc;
}

int pollModbusRequests() {
  uint16_t crc;
  uint16_t address;
  uint16_t length;
  uint16_t available_len;
  uint8_t fc;
  uint8_t id;
  int lengthIn;
  int lengthOut;

  available_len = Serial2.available();
  if (available_len != 0) {
    // Serial.print("Got some data\n");
    // if we have new data, update last received time and length.
    if (available_len != last_receive_len) {
      last_receive_len = available_len;
      last_receive_time = micros();
      return 0;
    }
    if (micros() < (last_receive_time + timeout)) { // if no new data, wait for T35 microseconds.
      return 0;
    }
    // we waited for the inter-frame timeout, read the frame.
    lengthIn = Serial2.readBytes(bufIn, MAX_BUFFER);
    last_receive_len = 0;
    last_receive_time = 0;
  } else {
    return 0;
  }

  if (bufIn[0] != unitID) return 0;
  if (lengthIn < 8) return 0;

  id = bufIn[0];
  fc = bufIn[1];
  address = mWord(bufIn[2], bufIn[3]); // first register.
  length = mWord(bufIn[4], bufIn[5]);  // number of registers to act upone or status.

  for (int i =  0; i < 5; i++) {
    Serial.printf("Got data in buf: %d , %d ", i, bufIn[i]);
  }

  if (fc == 4) { // Read input registers
    Serial.printf("\nRead input registers: id: %d, fc: %d, address: %d, length: %d, lengthIn: %d\n", id, fc, address, length, lengthIn);

    crc = word(bufIn[lengthIn - 1], bufIn[lengthIn - 2]);
    if (calcCRC(bufIn, lengthIn - 2) == crc) {
      Serial.print("CRC OK\n");
      // build valid empty answer.
      lengthOut = 3 + 2 * length + 2;
      bufOut[2] = 2 * length;

      // clear data out. Message example: 1, 4, 2, 0, 1, 120, 240
      memset(bufOut + 3, 0, bufOut[2]);
      bufOut[0] = unitID;
      bufOut[1] = fc;
      bufOut[2] = 2; // Status

      Serial.printf("Sending fake value: %d\n", fakeValue);
      bufOut[3] = (fakeValue >> 8) & 0xFF; // MSB
      bufOut[4] = fakeValue & 0xFF; // LSB
      if (++fakeValue > 400) {fakeValue = 0;}

      crc = calcCRC(bufOut, lengthOut - 2);
      bufOut[lengthOut - 2] = crc & 0xff;
      bufOut[lengthOut - 1] = crc >> 8;

      // Serial.printf("Awnser: ");
      // for (int i = 0; i < lengthOut; i++) {
      //   Serial.printf("%d : %d, ", i, bufOut[i]);
      // }
      // Serial.print("\n");

      digitalWrite(DE_PIN, HIGH);
      delay(100);
      Serial2.write(bufOut, lengthOut);
      Serial2.flush();
      digitalWrite(DE_PIN, LOW);
    }
  }
  return 1;
}

void loop() {
  pollModbusRequests();
}
