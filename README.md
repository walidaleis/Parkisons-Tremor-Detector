
# Parkinson's Tremor Detector
### Objective
This project aimed to use an Adafruit Circuit Playground Classic, which has an ATmega32U4 microcontroller, to detect a specific tremor characteristic of Parkinson's disease patients. Tracking tremors is important for physicians to monitor the overall condition of patients and adequately address the tremor given each patient's unique situation.

### Approach
We took a pretty standard approach, using a Fast Fourier Transform (FFT) with a windowing technique of 128 bins. We captured four windows and collected data from the accelerometer in each window, storing them in a 4x4 array. After those four windows, we averaged the data and used it to determine if there was a tremor present during that time, comparing with the established frequency range for Parkinson's of 3-6 Hz  in the literature .

We collected:
- The sum of all the frequencies in the window
- The ratio of all frequencies in the 3-6Hz band compared to all the frequencies in the window
- The ratio of the magnitudes in the 3-6Hz band compared to all the total magnitudes in the window
- The ratio of energy that is in the band compared to all the energy in the window

We then used the ratio and multiplied it by the scaling factor to determine the intensity of the tremor, lighting up the board's NeoPixels and playing a tone as needed.


References:
https://www.ncbi.nlm.nih.gov/pmc/articles/PMC7602495/


### Contributors
Cameron Bedard, Walid Al-Eisawi, Darren Kuo