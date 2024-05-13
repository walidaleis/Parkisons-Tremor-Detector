#include <Arduino.h>
#include <Adafruit_CircuitPlayground.h>
#include "Adafruit_ZeroFFT.h"
#include "arduinoFFT.h"

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

/*
----------- FFT Global Variables -----------
These values can be changed in order to evaluate the functions
*/
const uint16_t windowSamples = 256; // number of samples in the dataset
const double sampling = 200;        // sampling frequency in Hz
// const uint8_t amplitude = 4;      // amplitude of sinusoidal
const double startFrequency = 3; // Lower range of frequencies to test
const double stopFrequency = 6;  // higher range of frequencies to test
// const double step_size = 0.1;     // increment of frequency for each test loop

/*
These are the input and output vectors
Input vectors receive computed results from FFT
*/
double vReal[windowSamples];
double vImag[windowSamples];

// /* Create FFT object */
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, windowSamples, sampling);

unsigned long startTime; // used to measured the computation time of the FFT

void announceTremor(int intensity);

void determineTremor();

const int num_windows = 4;
const int num_metrics = 6;
// keeping track of the metrics in each window
float windowMetrics[num_windows - 1][num_metrics - 1];
// columns 0-3 in the array are windows
// rows 0-4 are metrics:
// row 0: standard deviation of time domain magnitudes
// row 1: average frequency is frequency domain
// row 2: most dominant frequency across 80% of the window
// row 3: peak amplitude in the 3-6Hz band
// row 4: energy content in the 3-6Hz band

int windowNum = 0;

void setupTimer()
{
  Serial.println("Starting timer");
  // 8 Bit Counter
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
  //         00000 - Unused
  //              0 - Disable interrupts on compare match B
  //               1 - Enable interrupts on compare match A
  //                0 - Disable overflow interrupts
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

  for (int *i = 0; *i < num_windows; i++)
  {
    for (int *j = 0; *j < num_metrics; j++)
    {
      num_metrics[i][j] = 0.0;
    }
  }
  // Initial Serial protocol
}

void loop()
{
  // Left button is pressed?
  if (PIND & (1 << PIND4))
  {
    // delay(1000);
    Serial.println("Left button pressed");

    // Turn on LED
    PORTC |= (1 << 7);

    // Start capture
    setupTimer();
  }

  // Right button pressed?
  if (PINF & (1 << PINF6))
  {
    // delay(1000);
    Serial.println("right button pressed");
    // Turn off LED
    PORTC &= ~(1 << 7);

    // Disable timer interrupts
    TIMSK0 = 0;

    // Clear array
  }
}

uint16_t samplesCounter = 0;
double windowSum = 0;
float X, Y, Z;
int count_to5 = 0;

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
    float magnitude = sqrt(X * X + Y * Y + Z * Z);

    windowSum += magnitude;

    if (false) // debug print statement
    {
      Serial.print("X: ");
      Serial.print(X);
      Serial.print("  Y: ");
      Serial.print(Y);
      Serial.print("  Z: ");
      Serial.println(Z);
    }

    vReal[samplesCounter] = magnitude;
    vImag[samplesCounter] = 0;
    samplesCounter++;
  }

  // Runs approximately every second
  if (samplesCounter == 256)
  {
    cli(); // stopping interrupt

    double windowMean = windowSum / 256;

    double runningSum = 0.0;

    for (float entry : vReal)
    {
      runningSum += (entry - windowMean) * (entry - windowMean);
    }
    windowMetrics[windowNum][0] = sqrt(runningSum / 256);

    samplesCounter = 0;
    Serial.println("5 seconds passed. Calculating FFT");
    delay(10);
    calculateFFT();

    windowNum++;
    if (windowNum > 3)
    {
      windowNum == 0;

      determineTremor();
    }
    sei(); // reenabling interrupt
  }
}

/*
one to three minutes of windowing
80% of movements in each window are a tremor
*/
void calculateFFT()
{
  unsigned long startTime = millis();
  // tapering the signal edges to zero to make it more continuous and periodic
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward); /* Weigh data */

  // computing time domain into frequency domain. Results are stored back into vReal and vImag
  FFT.compute(FFTDirection::Forward); /* Compute FFT */

  // computes the complex numbers containing the different frequencies into magnitudes and stores them back into vReal
  FFT.complexToMagnitude(); /* Compute magnitudes */

  float sumFreq = 0;  // sum of all frequencies
  float sumMag = 0;   // sum of all magnitudes
  float peakMag = 0;  // maximum magnitude of any element
  float domFreq = 0;  // frequency of the highest amplitude signal
  float peakFreq = 0; // highest frequency in the window
  float sumEnergy = 0;

  float bandPeakMag = 0; // max magnitude just in our band
  float bandMagSum = 0;  // sum of magnitudes in our band

  float bandEnergy = 0;

  for (int i = 0; i < windowSamples / 2; i++)
  {
    // calculating the freq for this element
    float freq = (float)i * (sampling / windowSamples);
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
      peakMag = mag;  // updating the max magnitude recorded
      domFreq = freq; // updating the frequency at that maximum magnitude
    }
    // calculating the new peak Frequency
    if (freq > peakFreq)
    {
      peakFreq = freq;
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
  // columns 0-3 in the array are windows
  // rows 0-4 are metrics:
  // row 0: standard deviation of time domain magnitudes
  // row 1: average frequency
  // row 2: percent of the entire window that is the freqs in the tremor band
  // row 3: peak amplitude in the 3-6Hz band
  // row 4: energy content in the 3-6Hz band
  // row 5: percent energy is from the tremor band

  float averageFreq = 0;
  float bandFreqPercent = 0;
  // if our sum of magnitudes is not zero to avoid dividing by zero
  if (sumMag != 0)
  {
    averageFreq = sumFreq / sumMag;

    bandFreqPercent = bandMagSum / sumMag;

    windowMetrics[windowNum][1] = averageFreq;
    windowMetrics[windowNum][2] = bandFreqPercent;

    //this will get deleted because if sumMag is not zero, sumEnergy is not zero but it's here for testing
    if (sumEnergy != 0)
      windowMetrics[windowNum][5] = bandEnergy / sumEnergy;
  }
  else
  {
    windowMetrics[windowNum][1] = 0;
    windowMetrics[windowNum][2] = 0;
    windowMetrics[windowNum][5] = 0;
  }
  windowMetrics[windowNum][3] = bandPeakMag;
  windowMetrics[windowNum][4] = bandEnergy;

  unsigned long endTime = millis();

  Serial.println("Window " + windowNum + ':');
  Serial.println("Stdev\tAvgFreq\tBandFreqPercent\tBandPeakMag\tBandEnergy");
  Serial.println("=======================================\n");
  Serial.print(windowMetrics[windowNum][0] + ',' + windowMetrics[windowNum][1] + ',' + windowMetrics[windowNum][2] + ',' + windowMetrics[windowNum][3] + ',' + windowMetrics[windowNum][4] + ',' + windowMetrics[windowNum][5]);
  Serial.print("Execution Duration: ");
  Serial.print(endTime - startTime);
  Serial.println(" ms");
}

void determineTremor()
{
  /*
  columns 0-3 in the array are windows
  rows 0-4 are metrics:
  row 0: standard deviation of time domain magnitudes
  row 1: average frequency
  row 2: percent of the entire window that is the freqs in the tremor band
  row 3: peak amplitude in the 3-6Hz band
  row 4: energy content in the 3-6Hz band
  windowMetrics[windowNum][0] = Stdev;
  windowMetrics[windowNum][1] = averageFreq;
  windowMetrics[windowNum][2] = bandFreqPercent;
  windowMetrics[windowNum][5] = bandEnergy / sumEnergy;
  windowMetrics[windowNum][3] = bandPeakMag;
  windowMetrics[windowNum][4] = bandEnergy;

  tremor if:
    avg freq is in the band
    percent in band is over 80%

  determine intensity:
    peak amplitude
    energy content
    energy percent is from band
  */
 float windowAvgFreq = 0;
}

void announceTremor(int intensity)
{
  // Serial.print("Announcing Tremor Intensity");
  CircuitPlayground.setPixelColor(0, 255, 0, 0);
  CircuitPlayground.setPixelColor(1, 128, 128, 0);
  // CircuitPlayground.setPixelColor(2, 0, 255, 0);
  // CircuitPlayground.setPixelColor(3, 0, 128, 128);
  // CircuitPlayground.setPixelColor(4, 0, 0, 255);
}