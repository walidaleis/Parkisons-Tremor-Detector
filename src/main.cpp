#include <Arduino.h>
#include <Adafruit_CircuitPlayground.h>
#include "Adafruit_ZeroFFT.h"
#include "arduinoFFT.h"

/*
These values can be changed in order to evaluate the functions
*/
const uint16_t samples = 64;
const double sampling = 66.666;
// const double startFrequency = 3.5;
// const double stopFrequency = 7.5;
// const double step_size = 0.1;

uint16_t samplesCounter = 0;
float X, Y, Z;

int count_to15 = 0;
// int data_count = 0;

// /*
// These are the input and output vectors
// Input vectors receive computed results from FFT
// */
double vReal[samples];
double vImag[samples];

// /* Create FFT object */
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, samples, sampling);

unsigned long startTime;

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

void announceTremor(int intensity);

void setupTimer()
{
  Serial.println("Starting timer");
  // Using Timer0 to trigger a fourier transform every five seconds
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

  // Initial Serial protocol
}

/*
one to three minutes of windowing
80% of movements in each window are a tremor
*/
void calculateFFT()
{
  cli();

  // Serial.println("Frequency\tDetected\ttakes (ms)");
  // Serial.println("=======================================\n");

  // delay(25);

  // for (double frequency = startFrequency; frequency <= stopFrequency; frequency += step_size)
  // {
  // startTime = millis();

  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward); /* Weigh data */

  FFT.compute(FFTDirection::Forward); /* Compute FFT */

  FFT.complexToMagnitude(); /* Compute magnitudes */

  // int peakMagnitude = 0;

  // for (uint16_t i = 0; i < samples / 2; i++)
  // {
  //   double currentFrequency = (i * sampling) / samples; // Calculate the actual frequency of the ith sample
  //   if (currentFrequency >= startFrequency && currentFrequency <= stopFrequency)
  //   {
  //     if (vReal[i] > peakMagnitude) // Check if this is the highest magnitude within the tremor range
  //     {
  //       peakMagnitude = vReal[i];
  //     }
  //   }
  // }
  // announceTremor(peakMagnitude);

  double x = FFT.majorPeak();
  // look at the bins individually and exclude the low freqs

  // Serial.print(frequency);
  Serial.print("frequency detected: ");
  Serial.print(x, 4);
  Serial.println();
  // for (int i = 0; i<samples; i++) {
  //   Serial.println(vReal[i]);
  // }
  // Serial.print("\t\t");
  // Serial.print(millis() - startTime);
  // Serial.println(" ms");
  // delay(2000); /* Repeat after delay */
  // }

  sei();
}

void announceTremor(int intensity)
{
  // Serial.print("Annoucning Tremor Intensity");
  CircuitPlayground.setPixelColor(0, 255, 0, 0);
  CircuitPlayground.setPixelColor(1, 128, 128, 0);
  // CircuitPlayground.setPixelColor(2, 0, 255, 0);
  // CircuitPlayground.setPixelColor(3, 0, 128, 128);
  // CircuitPlayground.setPixelColor(4, 0, 0, 255);
}

// Runs every ms
ISR(TIMER0_COMPA_vect)
{
  // Serial.print("interrupt");

  count_to15++;
  //data_count++;
  if (count_to15 == 15) // Counts up to 5ms
  {
    // Reset counter
    count_to15 = 0;
    // Record accelerometer data
    X = CircuitPlayground.motionX();
    Y = CircuitPlayground.motionY();
    Z = CircuitPlayground.motionZ();

    // Get the "average" as the root of the sum of squares of all dimesnions
    float a = sqrt(X * X + Y * Y + Z * Z);

    // remove later
    // Serial.print("X: ");
    // Serial.print(X);
    // Serial.print("  Y: ");
    // Serial.print(Y);
    // Serial.print("  Z: ");
    // Serial.println(Z);

    // delay(1000);
    // Serial.print("adding data to array");

    vReal[samplesCounter] = a;
    vImag[samplesCounter] = 0;

    samplesCounter++;
  }

  // Runs approximately every second
  if (samplesCounter == 64)
  { // Timer for 5S, then run FFT
    // X = CircuitPlayground.motionX();
    // Y = CircuitPlayground.motionY();
    // Z = CircuitPlayground.motionZ();
    // Serial.print("X: ");
    // Serial.print(X);
    // Serial.print("  Y: ");
    // Serial.print(Y);
    // Serial.print("  Z: ");
    // Serial.println(Z);

    //data_count = 0;
    samplesCounter = 0;
    calculateFFT();
    // Serial.println("5s passed");
  }
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