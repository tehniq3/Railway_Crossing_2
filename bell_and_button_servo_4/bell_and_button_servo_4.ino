
/*
 * bell_and_button
 *
 * Plays 8-bit PCM audio on pin 11 using pulse-width modulation (PWM)
 *
 * Uses two timers. The first changes the sample value 8000 times a second.
 * The second holds pin 11 high for 0-255 ticks out of a 256-tick cycle,
 * depending on sample value. The second timer repeats 62500 times per second
 * (16000000 / 256), much faster than the playback rate (8000 Hz), so
 * it almost sounds halfway decent, just really quiet on a PC speaker.
 *
 * Takes over Timer 1 (16-bit) for the 8000 Hz timer. This breaks PWM
 * (analogWrite()) for Arduino pins 9 and 10. Takes Timer 2 (8-bit)
 * for the pulse width modulation, breaking PWM for pins 11 & 3.
 *
 * sound sketch courtesy of Michael Smith <michael@hurts.ca>
 * See: https://playground.arduino.cc/Code/PCMAudio/
 * 
 * warning train bell crossing and hint for servo control: Mike Osborn (group: Arduino for Model Railroading)
 * custom sketch by Nicu Florica (niq_ro) <nicu.florica@gmail.com>
 * ver.2 - added servo movement
 * ver.3 - added led flashing
 * ver.4 - added 3rd led (tie lamp)
 */
 
// Bell setup
#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include "crossing_bell_8KHz_mono.h"  // This is file in the same folder as this sketch that contains the sound data (limited to 1-2 secs total sound due to limited Uno memory)
                                      // Sounds must be in C code which can bet obtained via editing in AUdacity and conversion from WAV files using various tools 
                                      //  (e.g. http://colinjs.com/software.htm#t_WAVToCode)
#define SAMPLE_RATE 8000   // 8 KHz sample rate; compromised between quality and limited PROGMEM space on Uno/Nano/etc.

// Use a cheap "earbud" headphone connected to pin 11 and ground for a modest sound level but still reasonable in scale. 
// Use an simple LM386 audio amp and cheap 8 ohm speaker to get a louder sound using the same speaker pin

const int ledPin = 13;     // Use the on-board Arduino LED to indicate on/off status of the bell sound for debugging speaker operation
const int speakerPin = 11; // Can be either 3 or 11, two PWM outputs connected to Timer 2; using 11 in this sketch
volatile uint16_t sample;
byte lastSample;
// END Bell setup

// Button setup
const int buttonPin = 2;    // pushbutton pin; connect pushbutton to buttonPin and ground
int ledState = HIGH;         // current state of the control variable (as well as the on-board LED as visual confirmation for bell-on state)
int buttonState;            // current reading from the button pin
int lastButtonState = LOW;  // previous reading from the input pin
unsigned long lastDebounceTime = 0;  // last time the button pin was toggled
unsigned long debounceDelay = 50;    // button debounce time
// END Button setup

// 750us = 0 degree, 1500us = 90 degree, 2250us = 180 degree
int unghi = 80;  // 1500us = 90 degree
int unghimin = 5;  // 750us = 0 degree
int unghimax = 80;  // 2250us = 180 degree
int tpcontrol = 0;
int tpcontrolmin = 750;
float deltacontrol = 150/18;  // 1500us/180degree

int deltaunghi = 1;
int semn  = 1;
unsigned long pauza = 10;
unsigned long tpmiscare;
#define servoPin 9

int soundControl = 0;
unsigned long tpapasare;
unsigned long tpsunet = 3000;  // 

#define LED1_PIN 3
#define LED2_PIN 4
#define LED3_PIN 5
#define BLINK_SPEED         400 // [ms] smaller number is faster blinking
unsigned long time_to_blink;
byte led1, led2;

// The following code get the sound data and, via user-controlled "ledState," may zero-out the data for the "no sound" case
// This code called at 8000 Hz to get the next sound sample.
ISR(TIMER1_COMPA_vect) {
    if (sample >= sounddata_length) {
        if (sample == sounddata_length + lastSample) {sample = 0;}
        else {OCR2B = sounddata_length + lastSample - sample;  }
    }
    else {OCR2A = soundControl * pgm_read_byte(&sounddata_data[sample]);} 

    ++sample;   }

void startPlayback()
{
    pinMode(speakerPin, OUTPUT);

    // Set up Timer 2 to do pulse width modulation on the speaker pin.

    // Use internal clock (datasheet p.160)
    ASSR &= ~(_BV(EXCLK) | _BV(AS2));

    // Set fast PWM mode  (p.157)
    TCCR2A |= _BV(WGM21) | _BV(WGM20);
    TCCR2B &= ~_BV(WGM22);

        // Do non-inverting PWM on pin OC2A (p.155)
        // On the Arduino this is pin 11.
        TCCR2A = (TCCR2A | _BV(COM2A1)) & ~_BV(COM2A0); TCCR2A &= ~(_BV(COM2B1) | _BV(COM2B0));
        // No prescaler (p.158)
        TCCR2B = (TCCR2B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);

        // Set initial pulse width to the first sample.
        OCR2A = pgm_read_byte(&sounddata_data[0]);
   
    // Set up Timer 1 to send a sample every interrupt.
    cli();

    // Set CTC mode (Clear Timer on Compare Match) (p.133)
    // Have to set OCR1A *after*, otherwise it gets reset to 0!
    TCCR1B = (TCCR1B & ~_BV(WGM13)) | _BV(WGM12);  TCCR1A = TCCR1A & ~(_BV(WGM11) | _BV(WGM10));

    // No prescaler (p.134)
    TCCR1B = (TCCR1B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);

    // Set the compare register (OCR1A).
    // OCR1A is a 16-bit register, so we have to do this with
    // interrupts disabled to be safe.
    OCR1A = F_CPU / SAMPLE_RATE;    // 16e6 / 8000 = 2000

    // Enable interrupt when TCNT1 == OCR1A (p.136)
    TIMSK1 |= _BV(OCIE1A);

    lastSample = pgm_read_byte(&sounddata_data[sounddata_length-1]); sample = 0; sei();
}

void setup()
{
    Serial.begin(9600);
    Serial.println();
    pinMode(servoPin, OUTPUT);
    pinMode(ledPin, OUTPUT);
    pinMode(LED1_PIN, OUTPUT);
    pinMode(LED2_PIN, OUTPUT);
    pinMode(LED3_PIN, OUTPUT);
    digitalWrite(LED1_PIN, 0);
    digitalWrite(LED2_PIN, 0);
    digitalWrite(LED3_PIN, 0);
    pinMode(buttonPin, INPUT_PULLUP);
    startPlayback();  // Bell sound playback initiated. Whether the bell sound is on/off is determined by "ledState" (see below)
}

void loop()
{
  // the entire void loop of this example sketch is designed to toggle the state of variable "soundControl" as the 
  // mechanism to turn the bell sound off and on. Put whatever sensor code you want in the main loop
  // to control the state of the bell via "soundControl" which is used in the "void setup" section to play/silence the bell
  digitalWrite(ledPin,ledState);
  int reading = digitalRead(buttonPin);  // Using a pushbutton to change on/off state of bell and LED

  // check to see if button pressed (i.e. the input went from LOW to HIGH) and debounce time satisfied)
  // If button state changed, from either noise or pressing, reset the debounce timer:
  if (reading != lastButtonState) { lastDebounceTime = millis(); }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the button state, it is longer than the debounce time so state is valid if the button state has changed:
    if (reading != buttonState) {
      buttonState = reading;

      // only toggle the ledPIN if the new button state is HIGH (i.e. user-input, in this exmaple sketch, pushbutton is placeholder for your own sensor (IR, etc.) and
      //  variable to change to bell off and on (in this example, that variable is "ledState")
      if (buttonState == HIGH) 
      {
        ledState = !ledState; 
        semn = 1;
        tpapasare = millis();
      }
    }
  }


if ((ledState == 0) and (millis()-tpmiscare > pauza))
{
   unghi = unghi + semn*deltaunghi;
 if (unghi > unghimax)
   {
    unghi = unghimax;
    semn = 0;
    soundControl = 0;
    Serial.println("Gate open");  // deschis
   }
    tpcontrol = tpcontrolmin + deltacontrol*unghi;
    if (unghi < unghimax)
    {
     Serial.print("Angle = ");
     Serial.print(unghi);  
     Serial.print("degree -> time = ");
     Serial.print(tpcontrol);
     Serial.println("us ");
    digitalWrite(servoPin, HIGH);
    delayMicroseconds(tpcontrol); // Duration of the pusle in microseconds
    digitalWrite(servoPin, LOW);
    }
    tpmiscare = millis(); 
}

if ((ledState == 1) and (millis()-tpmiscare > pauza))
{
   soundControl = 1;
if (millis() - tpapasare > tpsunet)
{ 
   unghi = unghi - semn*deltaunghi;
}
 if (unghi < unghimin)
   {
    unghi = unghimin;
    semn = 0;
    Serial.println("Gate closed");  // inchis
   }
     tpcontrol = tpcontrolmin + deltacontrol*unghi;
    if (unghi > unghimin)
    {
     Serial.print("Angle = ");
     Serial.print(unghi);  
     Serial.print("degree -> time = ");
     Serial.print(tpcontrol);
     Serial.println("us ");
    digitalWrite(servoPin, HIGH);
    delayMicroseconds(tpcontrol); // Duration of the pusle in microseconds
    digitalWrite(servoPin, LOW);
    }
    tpmiscare = millis(); 
}

  if(soundControl == 1) 
  {
    if(millis() > time_to_blink) 
    {
      time_to_blink = millis() + (unsigned long)BLINK_SPEED;
      led1 = !led1;
      led2 = !led1;
    }
  }
  else
  {
    led1 = 0;
    led2 = 0;
  }

    digitalWrite(LED1_PIN, led1);
    digitalWrite(LED2_PIN, led2);
    digitalWrite(LED3_PIN, (led1+led2)%2);
    
   lastButtonState = reading;  // save the button state
}
