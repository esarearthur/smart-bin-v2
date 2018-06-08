#include <printf.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <hcsr04.h>
#include <Servo.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

#define TRIG_PIN 4
#define ECHO_PIN 5

#define POWERON A0
#define ACT A1
#define BINFULL A2
#define HCSR04CTRL  A3

#define BIN_CAL_HEIGHT 35

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial

#define BAUD 115200
#define WHICH_NODE 2     // must be a number from 1 - 6 identifying the PTX node

const uint64_t wAddress[] = { 0xB3B4B5B6FFLL, 0xB3B4B5B6F1LL, 0xB3B4B5B6CDLL, 0xB3B4B5B6A3LL, 0xB3B4B5B60FLL, 0xB3B4B5B605LL };
const uint64_t PTXpipe = wAddress[ WHICH_NODE - 1 ];
float bin_height;
float bin_percentage;
int pos = 0;    // variable to store the servo position

RF24 radio(7, 6); // CE, CSN
HCSR04 hcsr04(TRIG_PIN, ECHO_PIN, 20, 4000);
Servo lockservo;

// This variable is made volatile because it is changed inside
// an interrupt function
volatile int f_wdt = 1;

// Watchdog Interrupt Service. This is executed when watchdog timed out.
ISR(WDT_vect) {
  if (f_wdt == 0) {
    // here we can implement a counter the can set the f_wdt to true if
    // the watchdog cycle needs to run longer than the maximum of eight
    // seconds.
    f_wdt = 1;
  }
}

// Enters the arduino into sleep mode.
void enterSleep(void)
{
  // There are five different sleep modes in order of power saving:
  // SLEEP_MODE_IDLE - the lowest power saving mode
  // SLEEP_MODE_ADC
  // SLEEP_MODE_PWR_SAVE
  // SLEEP_MODE_STANDBY
  // SLEEP_MODE_PWR_DOWN - the highest power saving mode
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();

  // Now enter sleep mode.
  sleep_mode();

  // The program will continue from here after the WDT timeout

  // First thing to do is disable sleep.
  sleep_disable();

  // Re-enable the peripherals.
  power_all_enable();
}

// Setup the Watch Dog Timer (WDT)
void setupWatchDogTimer() {
  // The MCU Status Register (MCUSR) is used to tell the cause of the last
  // reset, such as brown-out reset, watchdog reset, etc.
  // NOTE: for security reasons, there is a timed sequence for clearing the
  // WDE and changing the time-out configuration. If you don't use this
  // sequence properly, you'll get unexpected results.

  // Clear the reset flag on the MCUSR, the WDRF bit (bit 3).
  MCUSR &= ~(1 << WDRF);

  // Configure the Watchdog timer Control Register (WDTCSR)
  // The WDTCSR is used for configuring the time-out, mode of operation, etc

  // In order to change WDE or the pre-scaler, we need to set WDCE (This will
  // allow updates for 4 clock cycles).

  // Set the WDCE bit (bit 4) and the WDE bit (bit 3) of the WDTCSR. The WDCE
  // bit must be set in order to change WDE or the watchdog pre-scalers.
  // Setting the WDCE bit will allow updates to the pre-scalers and WDE for 4
  // clock cycles then it will be reset by hardware.
  WDTCSR |= (1 << WDCE) | (1 << WDE);

  /**
      Setting the watchdog pre-scaler value with VCC = 5.0V and 16mHZ
      WDP3 WDP2 WDP1 WDP0 | Number of WDT | Typical Time-out at Oscillator Cycles
      0    0    0    0    |   2K cycles   | 16 ms
      0    0    0    1    |   4K cycles   | 32 ms
      0    0    1    0    |   8K cycles   | 64 ms
      0    0    1    1    |  16K cycles   | 0.125 s
      0    1    0    0    |  32K cycles   | 0.25 s
      0    1    0    1    |  64K cycles   | 0.5 s
      0    1    1    0    |  128K cycles  | 1.0 s
      0    1    1    1    |  256K cycles  | 2.0 s
      1    0    0    0    |  512K cycles  | 4.0 s
      1    0    0    1    | 1024K cycles  | 8.0 s
  */
  WDTCSR  = (1 << WDP3) | (0 << WDP2) | (0 << WDP1) | (0 << WDP0);
  // Enable the WD interrupt (note: no reset).
  WDTCSR |= _BV(WDIE);
}

int getHeight() {
  digitalWrite(HCSR04CTRL, LOW);
  delay(100);
  int output = hcsr04.distanceInMillimeters();
  digitalWrite(HCSR04CTRL, HIGH);
  return output;
}

void setup() {
  Serial.begin(BAUD);
  while (!Serial);

  printf_begin();

  Serial.println("Initializing NRF24L01");
  radio.begin();
  radio.openReadingPipe(0, PTXpipe);  //open reading or receive pipe
  radio.setPALevel(RF24_PA_MIN);
  radio.setChannel(108);
  radio.printDetails();
  radio.stopListening(); //go into transmit mode
  Serial.println("Success NRF24L01");

  lockservo.attach(9);

  pinMode(HCSR04CTRL, OUTPUT);
  digitalWrite(HCSR04CTRL, LOW);

  setupWatchDogTimer();
  Serial.println("Initialisation complete.");
  delay(100);

  pinMode(POWERON, OUTPUT);
  pinMode(ACT, OUTPUT);
  pinMode(BINFULL, OUTPUT);

  digitalWrite(POWERON, HIGH);
  digitalWrite(ACT, LOW);
  digitalWrite(BINFULL, LOW);
}

uint16_t pdt = 0;
void loop() {
  // Wait until the watchdog have triggered a wake up.
  if (f_wdt != 1) {
    return;
  }

  while (millis() - pdt < 5000) {
    radio.startListening(); //switch to receive mode to see if the guess was right

    unsigned long startTimer = millis(); //start timer, we will wait 200ms
    bool timeout = false;
    while ( !radio.available() && !timeout ) { //run while no receive data and not timed out
      if (millis() - startTimer > 200 ) timeout = true; //timed out
    }

    if (timeout) {
      Serial.println("Timeout"); //no data to receive guess must have been wrong
      for (int j = 0; j < 10; ++j) {
        digitalWrite(ACT, digitalRead(ACT) == HIGH ? LOW : HIGH);
        delay(100);
      }
    }
    else  { //we received something so guess must have been right
      byte lidOpen; //variable to store received value
      radio.read( &lidOpen, 1); //read value
      if (lidOpen == 1) { //make sure it equals value we just sent, if so we are done
        // Servo Open
        lockservo.write(0);
      }
      else if (lidOpen == 0) { //make sure it equals value we just sent, if so we are done
        // Servo Close
        lockservo.write(90);
      }
    }
    delay(1000);
    radio.stopListening(); //go back to transmit mode
    digitalWrite(ACT, digitalRead(ACT) == HIGH ? LOW : HIGH);
    pdt = millis();
  }

  bin_height = getHeight() / 10;
  
  if (bin_height <= BIN_CAL_HEIGHT) {
    bin_percentage = 1 - (bin_height / BIN_CAL_HEIGHT);
    bin_percentage *= 100;

    radio.openWritingPipe(PTXpipe);        //open writing or transmit pipe

    if (!radio.write( &bin_percentage, sizeof(float) )) { //if the write fails let the user know over serial monitor
      Serial.print("Failed sending bin height: ");
      Serial.println(bin_percentage);
      digitalWrite(ACT, HIGH);
    }
    else { //if the write was successful
      digitalWrite(ACT, HIGH);
      Serial.print("Success sending bin height: ");
      Serial.println(bin_percentage);

      radio.startListening(); //switch to receive mode to see if the guess was right

      unsigned long startTimer = millis(); //start timer, we will wait 200ms
      bool timeout = false;
      while ( !radio.available() && !timeout ) { //run while no receive data and not timed out
        if (millis() - startTimer > 200 ) timeout = true; //timed out
      }

      if (timeout) {
        Serial.println("Timeout"); //no data to receive guess must have been wrong
        for (int j = 0; j < 10; ++j) {
          digitalWrite(ACT, digitalRead(ACT) == HIGH ? LOW : HIGH);
          delay(100);
        }
      }
      else  { //we received something so guess must have been right
        byte lidOpen; //variable to store received value
        radio.read( &lidOpen, 1); //read value
        if (lidOpen == 1) { //make sure it equals value we just sent, if so we are done
          // Servo Open
          lockservo.write(0);
        }
        else if (lidOpen == 0) { //make sure it equals value we just sent, if so we are done
          // Servo Close
          lockservo.write(90);
        }
      }
      delay(1000);
      radio.stopListening(); //go back to transmit mode
      digitalWrite(ACT, digitalRead(ACT) == HIGH ? LOW : HIGH);
    }

    // clear the flag so we can run above code again after the MCU wake up
    f_wdt = 0;
  }

  // Re-enter sleep mode.
  enterSleep();
  delay(1000);
}
