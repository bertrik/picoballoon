#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/crc16.h>
#include <SPI.h>
#include <RFM22.h>
#include <TinyGPS.h>

#define ASCII 7                 // ASCII 7 or 8
#define STOPBITS 2              // Either 1 or 2
#define TXDELAY 0               // Delay between sentence TX's
#define RTTY_BAUD 50            // Baud rate for use with RFM22B Max = 600
#define RADIO_FREQUENCY 434.160


#define RFM22B_SDN 8
#define RFM22B_PIN 10
#define LED 7

char callsign[8] = "pFALCON";   //Callsign

char datastring[80];
char txstring[80];
volatile int txstatus = 1;
volatile int txstringlength = 0;
volatile char txc;
volatile int txi;
volatile int txj;
unsigned int count = 0;

TinyGPS gps;
rfm22 radio1(RFM22B_PIN);

const int analogInPin = A0;
int sensorValue = 0;
float actualValue = 0.00;
float divider = 1.8;
char voltage[6];
int v1;
int v2;

float flat, flon = 0;
unsigned long age;
char latbuf[12] = "0", lonbuf[12] = "0", altbuf[12] = "0";
int hour = 0, minute = 0, second = 0, oldsecond = 0, sats = 0;
unsigned long date, time;
long int ialt = 123;

long int ticks = 1;

byte gps_set_sucess = 0;

bool newData = false;

bool cBusy = true;

bool reinit = false;
bool reinit_done = false;
int reinitcnt = 8;
int reinitcntr = 0;

void setup()
{
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  delay(500);
  digitalWrite(LED, HIGH);
  delay(500);

  initialise_interrupt();

  Serial.begin(9600);

  //Setup GPS
  uint8_t setNav[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00,
        0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA,
    0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC
  };

  uint8_t ecoMode[] =
      { 0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x00, 0x04, 0x1D, 0x85 };

  while (!gps_set_sucess) {
    sendUBX(setNav, sizeof(setNav) / sizeof(uint8_t));
    gps_set_sucess = getUBX_ACK(setNav);
    /*sendUBX(ecoMode, sizeof(ecoMode)/sizeof(uint8_t));
       gps_set_sucess&=getUBX_ACK(ecoMode); */
  }
  gps_set_sucess = 0;

  digitalWrite(LED, LOW);
  delay(500);
  digitalWrite(LED, HIGH);
  delay(500);
  digitalWrite(LED, LOW);
  delay(500);
  digitalWrite(LED, HIGH);
  delay(500);
  digitalWrite(LED, LOW);

  while (sats < 6 && flat == 0) {
    while (Serial.available()) {
      char c = Serial.read();
      newData = gps.encode(c);
      if (newData) {
        gps.f_get_position(&flat, &flon, &age);
        sats = gps.satellites();
      }
    }
  }

  setupGPSpower();
  delay(1000);
  //Setup RFM22B
  setupRadio();
}

void loop()
{
  unsigned long chars;
  unsigned short sentences, failed;

  while (Serial.available()) {
    char c = Serial.read();
    if (gps.encode(c))          // Did a new valid sentence come in?
      newData = true;
  }

  if (newData) {
    digitalWrite(LED, HIGH);
    gps.f_get_position(&flat, &flon, &age);
    sats = gps.satellites();
    dtostrf(flat, 10, 6, latbuf);
    dtostrf(flon, 9, 6, lonbuf);
    if (lonbuf[0] == ' ') {
      lonbuf[0] = '+';
    }
    if (latbuf[0] == ' ') {
      latbuf[0] = '+';
    }
    ialt = (gps.altitude() / 100);
    if (ialt >= 0) {
      itoa(ialt, altbuf, 10);
    }

    gps.get_datetime(&date, &time, &age);
    hour = (time / 1000000);
    minute = ((time - (hour * 1000000)) / 10000);
    second = ((time - ((hour * 1000000) + (minute * 10000))));
    second = second / 100;

    sensorValue = analogRead(analogInPin);
    actualValue = (sensorValue / 1023.00) * divider;
    v1 = actualValue;
    v2 = (actualValue - v1) * 100;
    snprintf(voltage, sizeof(voltage), "%i.%02i", v1, v2);

    cBusy = true;
    sprintf(datastring, "$$$$%s,%li,%02i:%02i:%02i,%s,%s,%s,%i,%s",
            callsign, ticks, hour, minute, second, latbuf, lonbuf, altbuf,
            sats, voltage);
    unsigned int CHECKSUM = gps_CRC16_checksum(datastring);     // Calculates the checksum for this datastring
    char checksum_str[7];
    sprintf(checksum_str, "*%04X\n", CHECKSUM);
    strcat(datastring, checksum_str);
    cBusy = false;

    gps.stats(&chars, &sentences, &failed);

  } else {
    digitalWrite(LED, LOW);
  }
  if (reinit == true) {
    setupRadio();
    reinit = false;
  }

}

ISR(TIMER1_COMPA_vect)
{
  switch (txstatus) {
  case 0:                      // This is the optional delay between transmissions.
    txj++;
    if (txj > (TXDELAY * RTTY_BAUD)) {
      txj = 0;
      txstatus = 1;
    }
    break;
  case 1:                      // Initialise transmission, take a copy of the string so it doesn't change mid transmission.
    if (reinit == false) {
      if (cBusy == false) {
        if (reinitcntr == reinitcnt) {
          reinit = true;
          reinitcntr = 0;
        } else {
          strcpy(txstring, datastring);
          txstringlength = strlen(txstring);
          if (txstringlength != 0)
            txj = 0;
          ticks++;
          txstatus = 2;
          reinitcntr++;
        }
      }
    }
    break;
  case 2:                      // Grab a char and lets go transmit it.
    if (txj < txstringlength) {
      txc = txstring[txj];
      txj++;
      txstatus = 3;
      rtty_txbit(0);            // Start Bit;
      txi = 0;
    } else {
      txstatus = 0;             // Should be finished
      txj = 0;
    }
    break;
  case 3:
    if (txi < ASCII) {
      txi++;
      if (txc & 1)
        rtty_txbit(1);
      else
        rtty_txbit(0);
      txc = txc >> 1;
      break;
    } else {
      rtty_txbit(1);            // Stop Bit
      txstatus = 4;
      txi = 0;
      break;
    }
  case 4:
    if (STOPBITS == 2) {
      rtty_txbit(1);            // Stop Bit
      txstatus = 2;
      break;
    } else {
      txstatus = 2;
      break;
    }

  }
}

void rtty_txbit(int bit)
{
  if (bit) {
    radio1.write(0x73, 0x03);   // High
  } else {
    radio1.write(0x73, 0x00);   // Low
  }
}

void setupGPSpower()
{
  //Set GPS ot Power Save Mode
  uint8_t setPSM[] = { 0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x01, 0x22, 0x92 };    // Setup for Power Save Mode (Default Cyclic 1s)

  sendUBX(setPSM, sizeof(setPSM) / sizeof(uint8_t));
}

void setupRadio()
{
  pinMode(RFM22B_SDN, OUTPUT);  // RFM22B SDN is on ARDUINO A3
  digitalWrite(RFM22B_SDN, LOW);
  delay(1000);
  rfm22::initSPI();
  radio1.init();
  radio1.write(0x71, 0x00);     // unmodulated carrier
  //This sets up the GPIOs to automatically switch the antenna depending on Tx or Rx state, only needs to be done at start up
  radio1.write(0x0b, 0x12);
  radio1.write(0x0c, 0x15);
  radio1.setFrequency(RADIO_FREQUENCY);
  radio1.write(0x6D, 0x04);     // turn tx low power 11db
  radio1.write(0x07, 0x08);
  delay(500);
}

uint16_t gps_CRC16_checksum(char *string)
{
  size_t i;
  uint16_t crc;
  uint8_t c;

  crc = 0xFFFF;

  // Calculate checksum ignoring the first four $s
  for (i = 4; i < strlen(string); i++) {
    c = string[i];
    crc = _crc_xmodem_update(crc, c);
  }

  return crc;
}

void initialise_interrupt()
{
  // initialize Timer1
  cli();                        // disable global interrupts
  TCCR1A = 0;                   // set entire TCCR1A register to 0
  TCCR1B = 0;                   // same for TCCR1B
  OCR1A = F_CPU / 1024 / RTTY_BAUD - 1; // set compare match register to desired timer count:
  TCCR1B |= (1 << WGM12);       // turn on CTC mode:
  // Set CS10 and CS12 bits for:
  TCCR1B |= (1 << CS10);
  TCCR1B |= (1 << CS12);
  // enable timer compare interrupt:
  TIMSK1 |= (1 << OCIE1A);
  sei();                        // enable global interrupts
}

// Send a byte array of UBX protocol to the GPS
void sendUBX(uint8_t * MSG, uint8_t len)
{
  for (int i = 0; i < len; i++) {
    Serial.write(MSG[i]);
    //mySerial.print(MSG[i], HEX);
  }
  Serial.println();
}

// Calculate expected UBX ACK packet and parse UBX response from GPS
boolean getUBX_ACK(uint8_t * MSG)
{
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();
  //mySerial.print(" * Reading ACK response: ");

  // Construct the expected ACK packet    
  ackPacket[0] = 0xB5;          // header
  ackPacket[1] = 0x62;          // header
  ackPacket[2] = 0x05;          // class
  ackPacket[3] = 0x01;          // id
  ackPacket[4] = 0x02;          // length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2];        // ACK class
  ackPacket[7] = MSG[3];        // ACK id
  ackPacket[8] = 0;             // CK_A
  ackPacket[9] = 0;             // CK_B

  // Calculate the checksums
  for (uint8_t i = 2; i < 8; i++) {
    ackPacket[8] = ackPacket[8] + ackPacket[i];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }

  while (1) {

    // Test for success
    if (ackByteID > 9) {
      // All packets in order!
      //mySerial.println(" (SUCCESS!)");
      return true;
    }
    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) {
      //mySerial.println(" (FAILED!)");
      return false;
    }
    // Make sure data is available to read
    if (Serial.available()) {
      b = Serial.read();

      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID]) {
        ackByteID++;
        //mySerial.print(b, HEX);
      } else {
        ackByteID = 0;          // Reset and look again, invalid order
      }

    }
  }
}
