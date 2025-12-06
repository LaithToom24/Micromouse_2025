sampling_time = 5e-3;
sampling_freq = 1/sampling_time;
cutoff_freq = 1.0;

% y[n] = b * y[n-1] + a * (x[n] + x[n-1])

b = (sampling_freq - pi*cutoff_freq)/(sampling_freq + pi*cutoff_freq);
a = pi * cutoff_freq/(sampling_freq + pi*cutoff_freq);

disp(sprintf("FIRST ORDER DIGITAL LOW-PASS FILTER WITH CUTOFF OF %.2f Hz AT SAMPLING TIME OF %.2f ms\ny[n] = %.4f y[n-1] + %.4f (x[n] + x[n-1])\n", cutoff_freq, sampling_time*1e3, b, a));

