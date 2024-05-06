#include <Arduino.h>
#include <Adafruit_CircuitPlayground.h>
#include "Adafruit_ZeroFFT.h"
#include "arduinoFFT.h"
// /*
// accelerator is center of board
//   - circled z is into the board

// Buttons

// Speaker for indicating tremor

// Slide switch

// NEO pixels

// Adafruit classic library
//   - Hello accelerometer for basic code to interface with accelerometer
//   - intensity meter
//   - color and intensity

// Plotting with Teleplot: Serial.print(">X: ");

// Teleplot shows a clear sine wave for a tremor

// Fourier transform will tell us the frequency after being fed the data

// Get things working with the libraries and then backdoor through the registers

// Don't use anything that #includes something we don't have

// Be vigilant about which timer we are using.

// Testing Ideas:
//   - Put the device on something spinning or tapping at six revolutions per second
//   - arduino fft driver

// Starting ideas:
//   - take readings 10 times a second
//   - check for a tremor in five second intervals
//   - implement the algorithm
//   - Schematic
//   - Plan out pseudo code
//   - Bandpass filter
//   - How to tell if it's not a tremor

// Plan:
//   - Left Button to start measuring
//     - lights to indicate it's running
//   - Right Button to stop and reset
//   - Speaker and neopixels to indicate a tremor

// FFT returns values that correspond to certain freqs
// */

/*
These values can be changed in order to evaluate the functions
*/
const uint16_t samples = 1000;
const double sampling = 40;
const uint8_t amplitude = 4;
const double startFrequency = 3.5;
const double stopFrequency = 7.5;
const double step_size = 0.1;

uint16_t samplesCounter = 0;

int count = 0;
int data_count = 0;

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

void checkButtons()
{
  // Left button is pressed?
  if (PIND & (1 << PIND4))
  {
    Serial.println("LEft button pressed");

    // Turn on LED
    PORTC |= (1 << 7);

    // Start capture
    setupTimer();
    delay(120);
  }

  // Right button pressed?
  if (PINF & (1 << PINF6))
  {
    Serial.println("right button pressed");
    // Turn off LED
    PORTC &= ~(1 << 7);

    // Disable timer interrupts
    TIMSK0 = 0;

    delay(120);

    // Clear array
  }
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

  Serial.println("Frequency\tDetected\ttakes (ms)");
  Serial.println("=======================================\n");

  delay(25);

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
    //look at the bins individually and exclude the low freqs

    Serial.print(frequency);
    Serial.print(": \t\t");
    Serial.print(x, 4);
    Serial.print("\t\t");
    Serial.print(millis() - startTime);
    Serial.println(" ms");
    // delay(2000); /* Repeat after delay */
  }

  sei();
}

void announceTremor(int intensity)
{
  //Serial.print("Annoucning Tremor Intensity");
  CircuitPlayground.setPixelColor(0, 255, 0, 0);
  CircuitPlayground.setPixelColor(1, 128, 128, 0);
  CircuitPlayground.setPixelColor(2, 0, 255, 0);
  CircuitPlayground.setPixelColor(3, 0, 128, 128);
  CircuitPlayground.setPixelColor(4, 0, 0, 255);
}

// Runs every ms
ISR(TIMER0_COMPA_vect)
{
  // Serial.print("interrupt");

  count++;
  if (count > 5) // Counts up to 5ms
  {
    // Reset counter
    count = 0;
    data_count++;
    // Record accelerometer data
    float X, Y, Z;
    X = CircuitPlayground.motionX();
    Y = CircuitPlayground.motionY();
    Z = CircuitPlayground.motionZ();

    // Get the "average" as the root of the sum of squares of all dimesnions
    float a = sqrt(X * X + Y * Y + Z * Z);

    //Serial.print("adding data to array");

    vReal[samplesCounter] = a;
    vImag[samplesCounter] = 0;

    if (data_count > 1000)
    { // Timer for 5S, then run FFT
      data_count = 0;
      samplesCounter = 0;
      calculateFFT();
      Serial.println("5s passed");
    }
  }
}

void loop()
{
  // Serial.print("loop");

  checkButtons();
  delay(100);
}