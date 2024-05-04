#include <Arduino.h>
#include <Adafruit_CircuitPlayground.h>
#include "Adafruit_ZeroFFT.h"
#include "arduinoFFT.h"
/*
accelerator is center of board
  - circled z is into the board

Buttons

Speaker for indicating tremor

Slide switch

NEO pixels

Adafruit classic library
  - Hello accelerometer for basic code to interface with accelerometer
  - intensity meter
  - color and intensity

Plotting with Teleplot: Serial.print(">X: ");

Teleplot shows a clear sine wave for a tremor

Fourier transform will tell us the frequency after being fed the data

Get things working with the libraries and then backdoor through the registers

Don't use anything that #includes something we don't have

Be vigilant about which timer we are using.

Testing Ideas:
  - Put the device on something spinning or tapping at six revolutions per second
  - arduino fft driver

Starting ideas:
  - take readings 10 times a second
  - check for a tremor in five second intervals
  - implement the algorithm
  - Schematic
  - Plan out pseudo code
  - Bandpass filter
  - How to tell if it's not a tremor

Plan:
  - Right Button to start measuring
    - lights to indicate it's running
  - Left Button to stop and reset
  - Speaker and neopixels to indicate a tremor

FFT returns values that correspond to certain freqs
*/

/*
These values can be changed in order to evaluate the functions
*/
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

void leftButtonFunction()
{
  // stops evaluating
}

void rightButtonFunction()
{
  // starts evaluating
}

void setup()
{
  Serial.begin(115200);
  while (!Serial);

  Serial.println("Ready");

  CircuitPlayground.begin();

  attachInterrupt(digitalPinToInterrupt(4), leftButtonFunction, CHANGE);

  attachInterrupt(digitalPinToInterrupt(19), rightButtonFunction, CHANGE);
}

void loop()
{
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
  TCCR1A = 0b00100000;
  //         00
  //           10, clear OCRB on compare match
  //             00 - C not used
  //               00 - Bits 1:0 0f (0100) CTC
  TCCR1B = 0b00001001;
  //         000 - Unused bits
  //            01 - Bits 32 of CTC
}

void announceTremor(int intensity)
{
  CircuitPlayground.setPixelColor(0, 255, 0, 0);
  CircuitPlayground.setPixelColor(1, 128, 128, 0);
  CircuitPlayground.setPixelColor(2, 0, 255, 0);
  CircuitPlayground.setPixelColor(3, 0, 128, 128);
  CircuitPlayground.setPixelColor(4, 0, 0, 255);
}