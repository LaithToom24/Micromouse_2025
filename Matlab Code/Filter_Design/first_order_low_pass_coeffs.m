sampling_time = 2e-3;
sampling_freq = 1/sampling_time;
cutoff_freq = 2;

% y[n] = b * y[n-1] + a * (x[n] + x[n-1])

b = (sampling_freq - pi*cutoff_freq)/(sampling_freq + pi*cutoff_freq)
a = pi * cutoff_freq/(sampling_freq + pi*cutoff_freq)

