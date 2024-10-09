clear all
close all
format long
clc

% Read the file
data = csvread('crutchLog.csv', 1, 0);

% Calculate the time difference
time_diff = diff(data(:, 1));
time_diff(1) = time_diff(2);

% Add the time difference as a new column
data = [ data(1:end-1, 1), time_diff, data(1:end-1, 2); data(end, 1), 0, data(end, 2)];

% Remove the last rows of data
data = data(1:end-2, :);

% Convert time from g to kg
data(:, 3) = data(:, 3) * 1e3;

% Convert time_diff from microseconds to miliseconds
data(:, 2) = data(:, 2) / 1e3;

% Convert time from microseconds to seconds
data(:, 1) = data(:, 1) / 1e6;

avg_time_diff = mean(data(:, 2));

% Plot the readings on the left axis
figure;
grid on
grid minor

yyaxis left;
plot(data(:, 1), data(:, 3));
xlabel('Time (s)');
ylabel('Readings (Kg)');

% Plot the time difference on the right axis
yyaxis right;
plot(data(:, 1), data(:, 2), '*--');
ylabel('Time Difference (ms)');

%plot avg
hold on
sz = size(data);
plot(data(:, 1), ones(sz(1))*avg_time_diff, 'k--');
text(max(data(:, 1)), max(data(:, 2)), sprintf('Avg Time Diff: %.2f ms\n Freq:%.2f Hz', avg_time_diff, 1000/avg_time_diff), 'HorizontalAlignment', 'right', 'VerticalAlignment', 'top');

% Set title and grid
title('Readings and Time Difference vs Time');
grid on;