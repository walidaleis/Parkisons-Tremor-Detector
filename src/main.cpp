#include <Arduino.h>
#include <Adafruit_CircuitPlayground.h>
#include "arduinoFFT.h"

// #define SCL_INDEX 0x00
// #define SCL_TIME 0x01
// #define SCL_FREQUENCY 0x02
// #define SCL_PLOT 0x03
#define DEBOUNCE_TIME 50

/*
----------- FFT Global Variables -----------
These values can be changed in order to evaluate the functions
*/
// const uint16_t windowSamples = 256; // number of samples in the dataset
// const uint16_t sampling = 200;      // sampling frequency in Hz
const uint8_t startFrequency = 3.0; // Lower range of frequencies to test
const uint8_t stopFrequency = 6.0;  // higher range of frequencies to test
const uint16_t windowSamples = 128;
uint16_t sampling = 64;

/*
These are the input and output vectors
Input vectors receive computed results from FFT
*/
double vReal[128];
double vImag[128];

ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, windowSamples, sampling);

// /* Create FFT object */

float startTime; // used to measured the computation time of the FFT

void announceTremor(uint8_t intensity);

void determineTremor();

void calculateFFT();

const uint8_t num_windows = 4;
const uint8_t num_metrics = 4;
// keeping track of the metrics in each window
static float windowMetrics[num_windows][num_metrics];


uint8_t windowNum = 0;

double lastDebounceTime = 0;

float alpha = 0.8;                 // smoothing factor for the low-pass filter
float gravity[3] = {0, 0, 0};      // store the estimated gravity components
float acceleration[3] = {0, 0, 0}; // store the filtered accelerometer readings

void setupTimer()
{
  TCCR0A = 0b00000010; // CTC count to OCR0A
   //         00 - Normal Port Operation
  //           00 - Normal Port Operation
  //             00 - Unused
  //               10 - Bits 1:0 of WGM (010)
  TCCR0B = 0b00000011;
  //         0000 - Unused
  //             0 - Bit 2 of WGM
  //              011 - Clk prescaler to 64
  OCR0A = 125; // TOP value
  TIMSK0 = 0b00000010;
}

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;

  CircuitPlayground.begin();

  Serial.println("Ready");

  // Set up left button (D4) and right button (F6) as inputs
  DDRD &= ~(1 << PIND4);
  DDRF &= ~(1 << PINF6);

  // Enable internal pull-up resistors
  PORTD |= (1 << PIND4);
  PORTF |= (1 << PINF6);

  // Enable LED
  DDRC |= (1 << PINC7);

  for (uint8_t i = 0; i < num_windows; i++)
  {
    for (uint8_t j = 0; j < num_metrics; j++)
    {
      windowMetrics[i][j] = 0.0;
    }
  }
  // Initial Serial protocol
}

void loop()
{
  unsigned long currentTime = millis();
  // Left button state check and debounce
  if (PIND & (1 << PIND4))
  {
    Serial.println("Left button pressed");
    PORTC |= (1 << 7);              // Turn on LED
    setupTimer();                   // Start capture
    lastDebounceTime = currentTime; // Update the debounce timer

    for (uint8_t i = 0; i < 10; i++)
    {
      CircuitPlayground.setPixelColor(i, 0, 255, 0); // Turn all LEDs green
    }
    delay(50);
    for (uint8_t i = 0; i < 10; i++)
    {
      CircuitPlayground.setPixelColor(i, 0, 0, 0); // Turn off all LEDs
    }
  }

  // Right button state check and debounce
  if (PINF & (1 << PINF6))
  {
    Serial.println("Right button pressed");
    PORTC &= ~(1 << 7);             // Turn off LED
    TIMSK0 = 0;                     // Disable timer interrupts
    lastDebounceTime = currentTime; // Update the debounce timer

    for (uint8_t i = 0; i < 10; i++)
    {
      CircuitPlayground.setPixelColor(i, 0, 255, 0); // Turn all LEDs green
    }
    delay(50);
    for (uint8_t i = 0; i < 10; i++)
    {
      CircuitPlayground.setPixelColor(i, 0, 0, 0); // Turn off all LEDs
    }
  }
}

uint16_t samplesCounter = 0;
float X, Y, Z;
uint8_t count_to5 = 0;

void updateAcceleration(float rawAccel[3])
{
  for (int i = 0; i < 3; i++)
  {
    gravity[i] = alpha * gravity[i] + (1 - alpha) * rawAccel[i];
    acceleration[i] = rawAccel[i] - gravity[i];
  }
}

ISR(TIMER0_COMPA_vect) // Runs every ms
{
  count_to5++;

  if (count_to5 == 5) // Counts up to 5ms
  {
    // Reset counter
    count_to5 = 0;

    // Record accelerometer data
    X = CircuitPlayground.motionX();
    Y = CircuitPlayground.motionY();
    Z = CircuitPlayground.motionZ();

    // Get the "average" as the root of the sum of squares of all dimensions
    float rawAccel[3] = {X, Y, Z};
    updateAcceleration(rawAccel);

    float magnitude = sqrt(acceleration[0] * acceleration[0] +
                           acceleration[1] * acceleration[1] +
                           acceleration[2] * acceleration[2]);

    vReal[samplesCounter] = magnitude;
    vImag[samplesCounter] = 0;
    samplesCounter++;
  }

  // Runs approximately every second
  if (samplesCounter == windowSamples)
  {
    cli(); // stopping interrupt

    samplesCounter = 0;

    calculateFFT();

    windowNum++;
    if (windowNum > 3)
    {
      windowNum = 0;

      determineTremor();
    }
    sei(); // reenabling interrupt
  }
}

void calculateFFT()
{
  //  tapering the signal edges to zero to make it more continuous and periodic
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward); /* Weigh data */

  // computing time domain into frequency domain. Results are stored back into vReal and vImag
  FFT.compute(FFTDirection::Forward); /* Compute FFT */

  // computes the complex numbers containing the different frequencies into magnitudes and stores them back into vReal
  FFT.complexToMagnitude(); /* Compute magnitudes */
  delay(25);
  // columns 0-3 in the array are windows
  // rows 0-4 are metrics:
  // row 0: average frequency in the data captured
  // row 1: ratio of frequencies in the entire window that is the 3-6Hz band
  // row 2: ratio of total magnitudes that is in the 3-6Hz band
  // row 3: ratio of energy in the 3-6Hz band compared to the entire window

  float sumFreq = 0;      // sum of all frequencies
  float sumMag = 0;       // sum of all magnitudes
  float peakMag = 0;      // maximum magnitude of any element
  float peakFreqBand = 0; // highest frequency in the window
  float sumEnergy = 0;
  float bandPeakMag = 0; // max magnitude just in our band
  float bandMagSum = 0;  // sum of magnitudes in our band
  float bandEnergy = 0;

  for (uint8_t i = 0; i < windowSamples / 2; i++)
  {
    // calculating the freq for this element
    float freq = (float)i * ((float)sampling / windowSamples);
    // magnitude of this element
    float mag = vReal[i];

    // calculating the sum of all freqs
    sumFreq += (freq * mag);
    // calculating sum of magnitudes
    sumMag += mag;
    // calculating sum of total energy
    sumEnergy += (mag * mag);

    // checking for peak Magnitude
    if (mag > peakMag)
    {
      peakMag = mag; // updating the max magnitude recorded
    }
    // calculating the new peak Frequency
    if (freq > peakFreqBand)
    {
      peakFreqBand = freq;
    }
    // checking if the current freq is in our band
    if (freq >= startFrequency && freq <= stopFrequency)
    {
      // calculating the sum of our bands magnitudes
      bandMagSum += mag;
      bandEnergy += (mag * mag);
      // checking if the current magnitude is greater than our current band peak magnitude
      if (mag > bandPeakMag)
      {
        bandPeakMag = mag; // assigning new peak magnitude in our tremor band
      }
    }
  }

  float averageFreq = 0;
  float bandFreqPercent = 0;
  // if our sum of magnitudes is not zero to avoid dividing by zero
  if (sumMag != 0)
  {
    averageFreq = sumFreq / sumMag;
    bandFreqPercent = bandMagSum / sumMag;
    windowMetrics[windowNum][0] = averageFreq;
    windowMetrics[windowNum][1] = bandFreqPercent;
    windowMetrics[windowNum][2] = bandEnergy / sumEnergy;
  }
  else
  {
    windowMetrics[windowNum][0] = 0;
    windowMetrics[windowNum][1] = 0;
    windowMetrics[windowNum][2] = 0;
  }
  windowMetrics[windowNum][3] = bandEnergy / sumEnergy;
}

void determineTremor()
{
  /*
    ---determine tremor---
    avg freq is in the band
    percent in band is over 30%

    ---determine intensity---
    Scales the ratio of 
  */
  float totalAvgFreq = 0;
  float percentInBand = 0;
  float totalMagnitudeInBand = 0;
  float totalEnergyPercentInBand = 0;


  for (uint8_t i = 0; i < num_windows; i++)
  {
    totalAvgFreq += windowMetrics[i][0];
    percentInBand += windowMetrics[i][1];
    totalMagnitudeInBand += windowMetrics[i][2];
    totalEnergyPercentInBand += windowMetrics[i][3];
  }
  totalAvgFreq /= num_windows;
  percentInBand /= num_windows;
  totalEnergyPercentInBand /= num_windows;
  totalMagnitudeInBand /= num_windows;

  if (totalAvgFreq >= startFrequency && totalAvgFreq <= stopFrequency && percentInBand > 0.25)
  {
    uint8_t intensity = totalEnergyPercentInBand * 70;
    if (intensity < 10)
    {
      for (uint8_t i = 0; i < 10; i++)
      {
        CircuitPlayground.setPixelColor(i, 0, 0, 0);
      }
      CircuitPlayground.setPixelColor(0, 255, 0, 0);
      CircuitPlayground.setPixelColor(9, 255, 0, 0);
      CircuitPlayground.playTone(1000, 200);
    }
    else if (intensity >= 10 && intensity < 20)
    {
      for (uint8_t i = 0; i < 10; i++)
      {
        CircuitPlayground.setPixelColor(i, 0, 0, 0);
      }
      CircuitPlayground.setPixelColor(0, 255, 0, 0);
      CircuitPlayground.setPixelColor(9, 255, 0, 0);
      CircuitPlayground.setPixelColor(1, 255, 0, 0);
      CircuitPlayground.setPixelColor(8, 255, 0, 0);
      CircuitPlayground.playTone(1200, 200);
      CircuitPlayground.playTone(1200, 200);
    }
    else if (intensity >= 20 && intensity < 30)
    {
      for (uint8_t i = 0; i < 10; i++)
      {
        CircuitPlayground.setPixelColor(i, 0, 0, 0);
      }
      CircuitPlayground.setPixelColor(0, 255, 0, 0);
      CircuitPlayground.setPixelColor(9, 255, 0, 0);
      CircuitPlayground.setPixelColor(1, 255, 0, 0);
      CircuitPlayground.setPixelColor(8, 255, 0, 0);
      CircuitPlayground.setPixelColor(2, 255, 0, 0);
      CircuitPlayground.setPixelColor(7, 255, 0, 0);
      CircuitPlayground.playTone(1400, 200);
      CircuitPlayground.playTone(1400, 200);
    }
    else if (intensity >= 30)
    {
      for (uint8_t i = 0; i < 10; i++)
      {
        CircuitPlayground.setPixelColor(i, 0, 0, 0);
      }
      CircuitPlayground.setPixelColor(0, 255, 0, 0);
      CircuitPlayground.setPixelColor(9, 255, 0, 0);
      CircuitPlayground.setPixelColor(1, 255, 0, 0);
      CircuitPlayground.setPixelColor(8, 255, 0, 0);
      CircuitPlayground.setPixelColor(2, 255, 0, 0);
      CircuitPlayground.setPixelColor(7, 255, 0, 0);
      CircuitPlayground.setPixelColor(3, 255, 0, 0);
      CircuitPlayground.setPixelColor(6, 255, 0, 0);
      CircuitPlayground.playTone(1600, 200);
      CircuitPlayground.playTone(1600, 200);
    }
  }
  else
  {
    for (uint8_t i = 0; i < 10; i++)
    {
      CircuitPlayground.setPixelColor(i, 0, 0, 0);
    }
  }
}