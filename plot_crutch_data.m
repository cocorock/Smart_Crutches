% Read the data from the CSV file
data = readtable('crutch_logs/combined_log_20251031_202950.csv');

% Convert system_time to datetime if it's not already
if ~isdatetime(data.system_time)
    data.system_time = datetime(data.system_time, 'InputFormat', 'yyyy-MM-dd HH:mm:ss.SSS');
end

% --- Time axis modification ---
% Calculate elapsed time in seconds from the start
startTime = min(data.system_time);
elapsedTime = seconds(data.system_time - startTime);
data.elapsed_time = elapsedTime;


% Scale the data
data.force = data.force / 100;
data.qw = data.qw / 1000;
data.qx = data.qx / 1000;
data.qy = data.qy / 1000;
data.qz = data.qz / 1000;

% --- User-defined thresholds for force data ---
min_force_threshold = -1000; % Adjust as needed
max_force_threshold = 4000;  % Adjust as needed

% Filter out anomalous data points from the force data
anomalous_indices = data.force < min_force_threshold | data.force > max_force_threshold;
data(anomalous_indices, :) = [];

% Get unique device names
devices = unique(data.device);

% Create a separate figure for each device
for i = 1:length(devices)
    device_data = data(strcmp(data.device, devices{i}), :);
    
    % Create a new figure for the current device
    figure('Name', ['Device: ' devices{i}]);

    % Plot Analog Data
    subplot(4,1,1);
    plot(device_data.elapsed_time, device_data.analog);
    title(['Analog Data for ' devices{i}]);
    xlabel('Time (s)');
    ylabel('Analog Value');
    legend(devices{i});

    % Plot Force Data
    subplot(4,1,2);
    plot(device_data.elapsed_time, device_data.force);
    title(['Force Data (Filtered) for ' devices{i}]);
    xlabel('Time (s)');
    ylabel('Force (grams)');
    legend(devices{i});

    % Plot Quaternion Data
    subplot(4,1,3);
    hold on;
    plot(device_data.elapsed_time, device_data.qw, 'DisplayName', 'qw');
    plot(device_data.elapsed_time, device_data.qx, 'DisplayName', 'qx');
    plot(device_data.elapsed_time, device_data.qy, 'DisplayName', 'qy');
    plot(device_data.elapsed_time, device_data.qz, 'DisplayName', 'qz');
    hold off;
    title(['Quaternion Orientation for ' devices{i}]);
    xlabel('Time (s)');
    ylabel('Value');
    legend('show');

    % Plot Euler Angles
    subplot(4,1,4);
    hold on;
    plot(device_data.elapsed_time, device_data.roll, 'DisplayName', 'Roll');
    plot(device_data.elapsed_time, device_data.pitch, 'DisplayName', 'Pitch');
    plot(device_data.elapsed_time, device_data.yaw, 'DisplayName', 'Yaw');
    hold off;
    title(['Euler Angles for ' devices{i}]);
    xlabel('Time (s)');
    ylabel('Degrees');
    legend('show');
end
