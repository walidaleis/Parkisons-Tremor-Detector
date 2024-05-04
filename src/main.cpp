#include <Arduino.h>
#include "arduinoFFT.h"
#include <Adafruit_CircuitPlayground.h>
#include <Adafruit_ZeroFFT.h>

const uint16_t samples = 64;
const double sampling = 40;
const uint8_t amplitude = 4;
const double startFrequency = 3;
const double stopFrequency = 6;
const double step_size = 0.1;

uint16_t samplesCounter = 0;

/*
These are the input and output vectors
Input vectors receive computed results from FFT
*/
double vReal[samples];
double vImag[samples];

/* Create FFT object */
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, samples, sampling);

unsigned long startTime;

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

void announceTremor(int intensity);



void setupButtons() {
  // Set left button (D4) and right button (F6) as inputs
  DDRD &= ~(1 << PIND4);
  DDRF &= ~(1 << PINF6);

  // Enable internal pull-up resistors
  PORTD |= (1 << PIND4);
  PORTF |= (1 << PINF6);
}

void setupLED() {
  DDRC |= (1 << PINC7);

}
void checkButtonsPressed() {
  // Left button is pressed?
  if (PIND & (1 << PIND4)) {
    // Turn on LED
    PORTC |= (1 << 7);

    // Start capture
    setupTimer();
  }

  // Right button pressed?
  if (PINF & (1 << PINF6)) {
    // Turn off LED
    PORTC &= ~(1 << 7);

    // Stop capture

  }
}


void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;

  Serial.println("Ready");

  CircuitPlayground.begin();

  setupButtons();

  attachInterrupt(digitalPinToInterrupt(4), leftButtonFunction, CHANGE);

  attachInterrupt(digitalPinToInterrupt(19), rightButtonFunction, CHANGE);

}

void loop() {
  checkButtonsPressed();
}

/*
one to three minutes of windowing
80% of movements in each window are a tremor
*/
void calculateFFT()
{
  Serial.println("Frequency\tDetected\ttakes (ms)");
  Serial.println("=======================================\n");

  for (double frequency = startFrequency; frequency <= stopFrequency; frequency += step_size)
  {
    startTime = millis();

    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward); /* Weigh data */

    FFT.compute(FFTDirection::Forward); /* Compute FFT */

    FFT.complexToMagnitude(); /* Compute magnitudes */

    int peakMagnitude = 0;

    for (uint16_t i = 0; i < samples / 2; i++)
    {
      double currentFrequency = (i * sampling) / samples; // Calculate the actual frequency of the ith sample
      if (currentFrequency >= startFrequency && currentFrequency <= stopFrequency)
      {
        if (vReal[i] > peakMagnitude) // Check if this is the highest magnitude within the tremor range
        {
          peakMagnitude = vReal[i];
        }
      }
    }
    announceTremor(peakMagnitude);

    double x = FFT.majorPeak();
    Serial.print(frequency);
    Serial.print(": \t\t");
    Serial.print(x, 4);
    Serial.print("\t\t");
    Serial.print(millis() - startTime);
    Serial.println(" ms");
    // delay(2000); /* Repeat after delay */
  }
}

ISR(TIMER0_COMPB_vect) // collecting data
{
  float X, Y, Z;
  X = CircuitPlayground.motionX();
  Y = CircuitPlayground.motionY();
  Z = CircuitPlayground.motionZ();
  
  float a = sqrt(X * X + Y * Y + Z * Z);

  if (samplesCounter > samples)
  {
    samplesCounter = 0;
  }
  vReal[samplesCounter] = a;
  vImag[samplesCounter] = 0;
}

ISR(TIMER1_COMPB_vect) // calculating FFT
{
  void calculateFFT();
}

void setupTimer()
{
  // Using Timer0 to trigger a fourier transform every five seconds
  // 8 Bit Counter
  TCCR0A = 0b00100010;
  //         00 - Normal Port Operation
  //           10 - Clear OC0B on compare match
  //             00 - Unused
  //               10 - Bits 1:0 of WGM (010)
  TCCR0B = 0b00000001;
  //         0000 - Unused
  //             0 - Bit 2 of WGM
  //              001 - No clk prescale
  OCR0A = 78;              // TOP value
  OCR0B = 0;               // Counter
  TIMSK0 |= (1 << OCIE0B); // Enables interrupt on compare B

  // 16 BIT TIMER
  // Sampling every 2.5 ms
  TCCR1A = 0b00100000;
  //         00
  //           10, clear OCRB on compare match
  //             00 - C not used
  //               00 - Bits 1:0 0f (0100) CTC
  TCCR1B = 0b00001001;
  //         000 - Unused bits
  //            01 - Bits 32 of CTC
  //              001 - No clk prescale
  OCR1A = 610; // Top Value
  OCR1B = 0;// Counter
  TIMSK1 |= (1 << OCIE1B); // Enables interrupt on compare B

}

void announceTremor(int intensity)
{
  CircuitPlayground.setPixelColor(0, 255, 0, 0);
  CircuitPlayground.setPixelColor(1, 128, 128, 0);
  CircuitPlayground.setPixelColor(2, 0, 255, 0);
  CircuitPlayground.setPixelColor(3, 0, 128, 128);
  CircuitPlayground.setPixelColor(4, 0, 0, 255);
}