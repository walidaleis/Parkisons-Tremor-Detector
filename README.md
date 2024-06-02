
# Parkinson's Tremor Detector

### Approach
We took a pretty standard approach, using a windowing technique of 128 bins. We captured four windows and collected data in each window, storing them in a 4x4 array. After those four windows, we averaged the data and used it to determine if there was a tremor present during that time.

We collected:
- The sum of all the frequencies in the window
- The ratio of all frequencies in the 3-6Hz band compared to all the frequencies in the window
- The ratio of the magnitudes in the 3-6Hz band compared to all the total magnitudes in the window
- The ratio of energy that is in the band compared to all the energy in the window

We then used the ratio and multiplied it by the scaling factor to determine the intensity of the tremor.


References:
https://www.ncbi.nlm.nih.gov/pmc/articles/PMC7602495/


### Contributors
Cameron Bedard, Walid Al-Eisawi, Darren Kuo