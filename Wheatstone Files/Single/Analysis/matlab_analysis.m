close all; clear all;
% Read the CSV file
data = readtable('muleta1 rate 1.csv');
% Extract the first column
F = data{:,1};

% Remove the offset
F = F-mean(F);

% Perform the DFT
F_dft = fft(F);

% Calculate the frequency axis
N = length(F); % Number of data points
Fs = 10/6; % Sampling frequency (change this if your data has a different sampling rate)
f = (0:N-1)*(Fs/N); % Frequency axis

% Create the frequency plot
figure;
plot(f, abs(F_dft));
title('Frequency Plot');
xlabel('Frequency (Hz)');
ylabel('Magnitude');

% Find the peak frequency
[~, idx] = max(abs(F_dft)); % Find the index of the max magnitude
peak_frequency = f(idx); % Find the corresponding frequency

% Display the peak frequency
disp(['The peak frequency is: ', num2str(peak_frequency), ' Hz']);

figure;
plot(F);
title('Signal Plot');
xlabel('Time');
ylabel('Amplitude');