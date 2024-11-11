% close all

%% first plot
% Define the data points
% weights = [0, 1745, 4265, 6000, 8155, 9318, 10375, 11752];
% readings = [-1335000, -1346021.00, -1367780.62, -1379707.87, -1382843.25, -1391240.62, -1396204.25, -1406132.75];

%Crutch BT-796 - Left
weights = [17250, 13940, 9750, 7526, 3780, 2230, 1130, 0];
readings = [-2084786.00, -2064025.62, -2039924.75, -2027374.37, -2005357.37, -1996114.25, -1987900.62, -1981253.25];

% % %Crutch BT-036 - Right
% weights = [17200, 14460, 11568, 7770, 5700, 2550, 1340, 0];
% readings = [-1430633.87, -1415796.87, -1400161.25, -1379589.87, -1368381.87, -1351532.62, -1345973.50, -1338805.87];

% Plot the original data
figure(1);
plot(weights, readings, 'o-', 'LineWidth', 2, 'DisplayName', 'Original Data');
xlabel('Weight (g)');
ylabel('Reading');
title('Weight vs. Reading');
grid on;
hold on;

% Initialize arrays to store slopes and intercepts
slopes = [];
intercepts = [];

% Perform linear interpolation and calculate line equations for each segment
for i = 1:length(weights)-1
    % Calculate slope and intercept for the line segment
    slope = (readings(i+1) - readings(i)) / (weights(i+1) - weights(i))
    intercept = readings(i) - slope * weights(i)
    
    % Store the slope and intercept
    slopes = [slopes, slope];
    intercepts = [intercepts, intercept];
    
    % Create a vector of weights with increments of 1.0g between each pair of points
    w = weights(i):1:weights(i+1);
    
    % Linear interpolation for readings
    r = slope * w + intercept;
    
    % Plot each segment separately
    plot(w, r, 'r-', 'LineWidth', 1, 'DisplayName', sprintf('y = %.2fx + %.2f', slope, intercept));
end

%% Linear interpolation using the first and last points
slope_full = (readings(end) - readings(1)) / (weights(end) - weights(1));
intercept_full = readings(1) - slope_full * weights(1);
w_full = weights(1):1:weights(end);
r_full = slope_full * w_full + intercept_full;

% Plot the full interpolation line
plot(w_full, r_full, 'b--', 'LineWidth', 2, 'DisplayName', sprintf('Full: y = %.2fx + %.2f', slope_full, intercept_full));

%% Linear interpolation using the second and last points
slope_partial = (readings(end) - readings(2)) / (weights(end) - weights(2));
intercept_partial = readings(2) - slope_partial * weights(2);
w_partial = weights(2):1:weights(end);
r_partial = slope_partial * w_partial + intercept_partial;

% Plot the partial interpolation line
plot(w_partial, r_partial, 'g--', 'LineWidth', 2, 'DisplayName', sprintf('Partial: y = %.2fx + %.2f', slope_partial, intercept_partial));

% Extend regression to a full range
w_partial = weights(1):1:weights(end);
r_partial = slope_partial * w_partial + intercept_partial;
plot(w_partial, r_partial, 'r--', 'LineWidth', 1, 'DisplayName', sprintf('Partial: y = %.2fx + %.2f', slope_partial, intercept_partial));

% Update legend with line equations
legend('Location', 'best');

hold off;

%% Create a new plot for the stored line equations
figure(2);
hold on;
title('Stored Line Equations from 0 to 12000g');
xlabel('Weight (g)');
ylabel('Reading');
grid on;

% Plot each stored line equation from 0 to 12000g
for i = 1:length(slopes)
    w_range = 0:1:12000;
    r_range = slopes(i) * w_range + intercepts(i);
    plot(w_range, r_range, 'LineWidth', 1.5, 'DisplayName', sprintf('Segment %d: y = %.2fx + %.2f', i, slopes(i), intercepts(i)));
end

% Update legend for the new plot
legend('Location', 'best');

hold off;
%% thrid plot
% Define the data points
% weights = [0, 1745, 4265, 5568, 8155, 9318, 10375, 11752];
% readings = [-1257623.62, -1346021.00, -1367780.62, -1379707.87, -1382843.25, -1391240.62, -1396204.25, -1406132.75];

% Plot the original data
figure(3);
plot(weights, readings, 'o', 'LineWidth', 2, 'DisplayName', 'Original Data');
xlabel('Weight (g)');
ylabel('Reading');
title('Weight vs. Reading with Polynomial Approximation');
grid on;
hold on;

% Fit a third-degree polynomial to the data
p = polyfit(weights, readings, 1);

% Generate a range of weights for plotting the polynomial curve
w_fit = linspace(min(weights), max(weights), 1000);
r_fit = polyval(p, w_fit);

% Plot the polynomial approximation
plot(w_fit, r_fit, 'r-', 'LineWidth', 2, 'DisplayName', sprintf(' Degree Poly: y = %.2fx^3 + %.2fx^2', p(1), p(2)));

% Update legend
legend('Location', 'best');

hold off;

%% new mapping plot 
figure(3)
hold on;
title('New mapping from 0 to 12000g');
xlabel('Weight (g)');
ylabel('Reading');
grid on;

w_range = 0:1:weights(2);
r_range = slopes(1) * w_range + intercepts(1);
plot(w_range, r_range, 'LineWidth', 1.5, 'DisplayName', sprintf('Segment %d: y = %.4fx + %.4f', 1, slopes(1), intercepts(1)));

w_range = weights(2):1:weights(end);
r_range = slope_partial* w_range + intercept_partial;
plot(w_range, r_range, 'LineWidth', 1.5, 'DisplayName', sprintf('Segment %d: y = %4fx + %.4f', 2, slope_partial, intercept_partial));

% Update legend
legend('Location', 'best');

hold off;