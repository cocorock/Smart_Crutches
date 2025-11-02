% Read the data from the CSV file
data = readtable('crutch_logs/combined_log_20251031_202950.csv');

% Convert system_time to datetime if it's not already
if ~isdatetime(data.system_time)
    data.system_time = datetime(data.system_time, 'InputFormat', 'yyyy-MM-dd HH:mm:ss.SSS');
end

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

% Create a new figure
figure;

% Plot Analog Data
subplot(4,1,1);
hold on;
for i = 1:length(devices)
    device_data = data(strcmp(data.device, devices{i}), :);
    plot(device_data.system_time, device_data.analog);
end
hold off;
title('Analog Data');
xlabel('Time');
ylabel('Analog Value');
legend(devices);

% Plot Force Data
subplot(4,1,2);
hold on;
for i = 1:length(devices)
    device_data = data(strcmp(data.device, devices{i}), :);
    plot(device_data.system_time, device_data.force);
end
hold off;
title('Force Data (Filtered)');
xlabel('Time');
ylabel('Force (grams)');
legend(devices);

% Plot Quaternion Data
subplot(4,1,3);
hold on;
for i = 1:length(devices)
    device_data = data(strcmp(data.device, devices{i}), :);
    plot(device_data.system_time, device_data.qw, 'DisplayName', [devices{i} ' qw']);
    plot(device_data.system_time, device_data.qx, 'DisplayName', [devices{i} ' qx']);
    plot(device_data.system_time, device_data.qy, 'DisplayName', [devices{i} ' qy']);
    plot(device_data.system_time, device_data.qz, 'DisplayName', [devices{i} ' qz']);
end
hold off;
title('Quaternion Orientation');
xlabel('Time');
ylabel('Value');
legend('show');

% Plot Euler Angles
subplot(4,1,4);
hold on;
for i = 1:length(devices)
    device_data = data(strcmp(data.device, devices{i}), :);
    plot(device_data.system_time, device_data.roll, 'DisplayName', [devices{i} ' Roll']);
    plot(device_data.system_time, device_data.pitch, 'DisplayName', [devices{i} ' Pitch']);
    plot(device_data.system_time, device_data.yaw, 'DisplayName', [devices{i} ' Yaw']);
end
hold off;
title('Euler Angles');
xlabel('Time');
ylabel('Degrees');
legend('show');