//#define DEBUG
#define TINY_GSM_MODEM_SIM800

#include <printf.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <TinyGsmClient.h>

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial

#include <SoftwareSerial.h>
SoftwareSerial SerialAT(2, 3); // RX, TX

#ifdef DEBUG
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

#define BAUD 115200
#define DTR A3

#define POWERON A0
#define ACT A1
#define BINFULL A2

#define RCVNUMBER "+233203930215"

RF24 radio(7, 6); // CE, CSN

const uint64_t rAddress[] = { 0x7878787878LL, 0xB3B4B5B6F1LL, 0xB3B4B5B6CDLL, 0xB3B4B5B6A3LL, 0xB3B4B5B60FLL, 0xB3B4B5B605LL };
float bin_height;

int unreadSMSLocs[20];
int unreadSMSNum;
bool SentSMS80 = false;
bool SentSMSFull = false;
struct SMSmessage sms;

byte daNumber = 0; //The number that the transmitters are trying to guess

enum __MODE {
  MASTER = 0,
  READ_BIN_01 = 1,
  READ_BIN_02 = 2
};

void ModemSleep() {
  modem.sendAT(GF("+CSCLK=1"));
  digitalWrite(DTR, HIGH);
  if (modem.waitResponse(5000L) != 1) {
    Serial.println(F("Modem sleep activated"));
  }
}

void ModemWakeup() {
  digitalWrite(DTR, LOW);
  delay(10);
  modem.sendAT();
  delay(20);
  modem.sendAT(GF("+CSCLK=0"));
  if (modem.waitResponse(5000L) != 1) {
    Serial.println(F("Modem sleep deactivated"));
  }
}

void setup() {
  pinMode(A3, OUTPUT);
  digitalWrite(A3, LOW);

  Serial.begin(BAUD);
  while (!Serial);

  // Set GSM module baud rate
  SerialAT.begin(9600);
  delay(3000);

  printf_begin();

  // Restart takes quite some time
  SerialMon.println("Initializing modem...");
  modem.restart();

  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem: ");
  SerialMon.println(modemInfo);

  SerialMon.print("Waiting for network...");
  if (!modem.waitForNetwork()) {
    SerialMon.println(" fail");
    while (true);
  }
  SerialMon.println("Modem initialization complete");

  modem.sendAT(GF("+CMGF=1"));
  if (modem.waitResponse(5000L) != 1) {
    Serial.println(F("Changing from PDU to TEXT mode failed"));
  }
  modem.sendAT(GF("+CPMS=\"SM\""));
  if (modem.waitResponse(5000L, GF("+CPMS:")) != 1) {
    Serial.println(F("Changing SMS location to SIM failed"));
  }

  /*if(modem.sendSMS("+233267766253", "HELLO"))
    Serial.println("Sent SMS");
    else
    Serial.println("Failed");*/

  Serial.println("Initializing NRF24L01");
  radio.begin();
  //radio.openWritingPipe(addresses[MASTER]);
  radio.openReadingPipe(0, rAddress[0]);
  radio.openReadingPipe(1, rAddress[1]);
  radio.openReadingPipe(2, rAddress[2]);
  radio.openReadingPipe(3, rAddress[3]);
  radio.openReadingPipe(4, rAddress[4]);
  radio.openReadingPipe(5, rAddress[5]);
  radio.setPALevel(RF24_PA_MIN);
  radio.setChannel(108);
  radio.printDetails();
  radio.startListening();
  Serial.println("Success NRF24L01");

  pinMode(POWERON, OUTPUT);
  pinMode(ACT, OUTPUT);
  pinMode(BINFULL, OUTPUT);

  digitalWrite(POWERON, HIGH);
  digitalWrite(ACT, LOW);
  digitalWrite(BINFULL, LOW);
}

unsigned long pdt = 0;

void loop() {
  byte pipeNum = 0; //variable to hold which reading pipe sent data
  byte gotByte = 0; //used to store payload from transmit module
  String SMS;

  if (millis() - pdt > 5000) {
    memset(unreadSMSLocs, 0x00, sizeof(unreadSMSLocs));
    unreadSMSNum = modem.getUnreadSMSLocs(unreadSMSLocs);
    if (unreadSMSNum > 0) {
      for (int j = 0; j < unreadSMSNum; ++j) {
        sms = modem.readSMS(unreadSMSLocs[j]);
        Serial.print("New message: ");
        Serial.println(sms.message);
      }

      int retry = 0;

      if (sms.message.indexOf("OPEN B1") >= 0) {
        while (retry < 3) {
          // Open Bin 1
          if (sendToXmitter(1, 1)) {
            Serial.println("Unlock bin"); //if true we successfully responded
            retry = 3;
            SentSMS80 = false;
            SentSMSFull = false;
          }
          else {
            Serial.println("Write failed"); //if true we failed responding
            ++retry;
          }

          // Close Bin 2
          if (sendToXmitter(2, 0)) {
            Serial.println("Lock bin"); //if true we successfully responded
            retry = 3;
            SentSMS80 = false;
            SentSMSFull = false;
          }
          else {
            Serial.println("Write failed"); //if true we failed responding
            ++retry;
          }
          delay(100);
        }
      }
      else if (sms.message.indexOf("OPEN B2") >= 0) {
        while (retry < 3) {
          // Close Bin 1
          if (sendToXmitter(1, 0)) {
            Serial.println("Lock bin"); //if true we successfully responded
            retry = 3;
            SentSMS80 = false;
            SentSMSFull = false;
          }
          else {
            Serial.println("Write failed"); //if true we failed responding
            ++retry;
          }

          // Open Bin 2
          if (sendToXmitter(2, 1)) {
            Serial.println("Unlock bin"); //if true we successfully responded
            retry = 3;
            SentSMS80 = false;
            SentSMSFull = false;
          }
          else {
            Serial.println("Write failed"); //if true we failed responding
            ++retry;
          }
          delay(100);
        }
      }
    }
    pdt = millis();
  }

  while (radio.available(&pipeNum)) { //Check if received data
    //radio.read( &gotByte, 1 ); //read one byte of data and store it in gotByte variable
    radio.read(&bin_height, sizeof(float));
    Serial.print("Received data from transmitter: ");
    Serial.println(pipeNum + 1); //print which pipe or transmitter this is from
    Serial.print("Refuse height: ");
    Serial.println(bin_height); //print payload or the number the transmitter guessed

    if (bin_height >= 0 && bin_height < 80) { //if true they guessed wrong
      Serial.print("Bin-");
      Serial.print(pipeNum + 1);
      Serial.println(" OK");

      if (sendToXmitter(pipeNum, 1))
        Serial.println("Unlock bin"); //if true we successfully responded
      else
        Serial.println("Write to NRF24L01 TX failed"); //if true we failed responding
    }
    else if (bin_height >= 80 && SentSMS80 == false) { //if this is true they guessed right
      SMS = "BIN-";
      SMS += String(pipeNum + 1);
      SMS += " Refuse height: 80%";

      if (modem.sendSMS(RCVNUMBER, SMS)) {
        Serial.println("Sent SMS");
        SentSMS80 = true;
      }
      else
        Serial.println("SMS Failed");
    }

    if (bin_height > 93 && SentSMSFull == false) {
      Serial.print("Bin-");
      Serial.print(pipeNum + 1);
      Serial.println(" full");

      if (sendToXmitter(pipeNum, 0)) {
        SMS = "BIN-";
        SMS += String(pipeNum + 1);
        SMS += " Refuse height: Full";

        Serial.println("Lock bin"); //if true we successfully responded
        if (modem.sendSMS(RCVNUMBER, SMS)) {
          Serial.println("Sent SMS");
          SentSMSFull = true;
        }
        else
          Serial.println("SMS Failed");
      }
      else
        Serial.println("Write to NRF24L01 TX failed"); //if true we failed responding
    }
    Serial.println();
  }
}

//This function turns the receiver into a transmitter briefly to tell one of the nRF24s
//in the network that it guessed the right number. Returns true if write to module was
//successful
bool sendToXmitter(byte xMitter, byte lid) {
  bool worked; //variable to track if write was successful
  radio.stopListening(); //Stop listening, start receiving data.
  radio.openWritingPipe(rAddress[xMitter]); //Open writing pipe to the nRF24 that guessed the right number
  // note that this is the same pipe address that was just used for receiving
  if (!radio.write(&lid, 1))  worked = false; //write the correct number to the nRF24 module, and check that it was received
  else worked = true; //it was received
  radio.startListening(); //Switch back to a receiver
  return worked;  //return whether write was successful
}
