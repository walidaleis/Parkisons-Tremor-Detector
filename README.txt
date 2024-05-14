We took a pretty standard approach. We did a windowing technique of 128 bins. We captured four windows and collected data on each window, storing them in a 4x4 array. After four windows, we averaged the data and used it to determine if there was a tremor present during that time.

We collected:
The sum of all the freqs in the window.
The ratio of all freqs that is in the 3-6Hz compared to all the freqs in the window
Ratio of the magnitudes in the 3-6Hz band compared to all the total magnitudes in the window
The ratio of energy that is in the band compared to all the energy in the window

We then used the ratio and multiplied it by the scaling factor to determine the intensity of the tremor.


sources:
https://www.ncbi.nlm.nih.gov/pmc/articles/PMC7602495/
