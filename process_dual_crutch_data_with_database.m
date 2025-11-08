% MATLAB Script for Dual Crutch Data Analysis with Segment Storage
% Reads combined CSV file, plots data, segments gaits, and saves/appends segments
% Author: Created for Crutch HMI System
% Date: 2025
% Features: 
%   - Automatic time_cuts selection based on filename
%   - Segments normalized to 0-1 (0% to 100% of gait cycle)
%   - Appends segments to MAT file for cumulative analysis
 
clear all;
close all;
clc;

%% ========================================================================
% CONFIGURATION - TIME CUTS DATABASE
% =========================================================================

% Define all time cuts for different files
% The script will automatically select the appropriate one based on filename
time_cuts_database = {
    '20251105_002950.csv', [14.72, 18.925, 23.298, 27.434, 31.939, 36.389, 41.05];
    '20251105_003116.csv', [14.33, 19.46, 24.11, 29.03];
    '20251105_003627.csv', [14.9, 19.85, 24.7, 30.07, 35.18, 40.35, 46.09];
    '20251105_003835.csv', [15.21, 20.77, 26.74, 31.94, 37.29, 42.3, 47.81];
    '20251105_004518.csv', [18.19, 22.94, 29.17, 33.42, 38.578, 43.97, 49.47];
    '20251105_004706.csv', [14.63, 20.32, 26.18, 31.6, 36.86, 41.97, 47.97];
    '20251105_005111.csv', [13.82, 19.58, 25.15, 30.83];
};

% Output MAT file for storing all segments
segments_mat_file = 'all_gait_segments.mat';

%% ========================================================================
% LOAD DATA
% =========================================================================

% File selection
[filename, filepath] = uigetfile('*.csv', 'Select Combined Crutch Data CSV File');
if filename == 0
    error('No file selected');
end
fullpath = fullfile(filepath, filename);

fprintf('Loading data from: %s\n', filename);

% Find matching time_cuts for this file
time_cuts = [];
for i = 1:size(time_cuts_database, 1)
    if contains(filename, time_cuts_database{i, 1})
        time_cuts = time_cuts_database{i, 2};
        fprintf('??? Found matching time_cuts for file: %s\n', time_cuts_database{i, 1});
        break;
    end
end

% Error if no matching time_cuts found
if isempty(time_cuts)
    error(['ERROR: No time_cuts defined for file "%s"\n' ...
           'Please add time_cuts for this file to the time_cuts_database.\n' ...
           'Available files in database:\n  - %s'], ...
           filename, strjoin(time_cuts_database(:,1), '\n  - '));
end

% Number of segments (should be length(time_cuts) - 1)
num_segments = length(time_cuts) - 1;
fprintf('Will extract %d segments using time_cuts: [%s]\n', ...
    num_segments, num2str(time_cuts, '%.2f '));

% Read the CSV file
data = readtable(fullpath);

% Extract time and data for both crutches
time_aligned = data.AlignedTime_s_;

% Crutch 1 data
force1 = data.Crutch1_Force;
qw1 = data.Crutch1_Qw;
qx1 = data.Crutch1_Qx;
qy1 = data.Crutch1_Qy;
qz1 = data.Crutch1_Qz;

% Crutch 2 data
force2 = data.Crutch2_Force;
qw2 = data.Crutch2_Qw;
qx2 = data.Crutch2_Qx;
qy2 = data.Crutch2_Qy;
qz2 = data.Crutch2_Qz;

fprintf('Data loaded successfully!\n');
fprintf('Total samples: %d\n', length(time_aligned));
fprintf('Duration: %.2f seconds\n', max(time_aligned));

%% ========================================================================
% CALCULATE EULER ANGLES FROM QUATERNIONS
% =========================================================================

fprintf('Calculating Euler angles...\n');

% Crutch 1 Euler angles (Roll, Pitch, Yaw)
[roll1, pitch1, yaw1] = quat2euler(qw1, qx1, qy1, qz1);

% Crutch 2 Euler angles (Roll, Pitch, Yaw)
[roll2, pitch2, yaw2] = quat2euler(qw2, qx2, qy2, qz2);

%% ========================================================================
% FIGURE 1: FULL DATA WITH 3 PLOTS
% =========================================================================

fprintf('Creating Figure 1: Full data visualization...\n');

figure('Name', 'Full Crutch Data Analysis', 'Position', [100, 100, 1200, 900]);

% Plot 1: Forces
subplot(3, 1, 1);
plot(time_aligned, force1, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Crutch 1 (Right)');
hold on;
plot(time_aligned, force2, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Crutch 2 (Left)');
for i = 1:length(time_cuts)
    xline(time_cuts(i), 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
end
xlabel('Time (s)');
ylabel('Force (units)');
xlim([13.8, 30.82]);
title('Force Measurements');
legend('Location', 'best');
grid on;

% Plot 2: Quaternions
subplot(3, 1, 2);
plot(time_aligned, qw1, 'b-', 'DisplayName', 'C1 Qw', 'LineWidth', 1.5);
hold on;
plot(time_aligned, qx1, 'b--', 'DisplayName', 'C1 Qx', 'LineWidth', 1.5);
plot(time_aligned, qy1, 'b:', 'DisplayName', 'C1 Qy', 'LineWidth', 1.5);
plot(time_aligned, qz1, 'b-.', 'DisplayName', 'C1 Qz', 'LineWidth', 1.5);
% plot(time_aligned, qw2, 'r-', 'DisplayName', 'C2 Qw', 'LineWidth', 1.5);
% plot(time_aligned, qx2, 'r--', 'DisplayName', 'C2 Qx', 'LineWidth', 1.5);
% plot(time_aligned, qy2, 'r:', 'DisplayName', 'C2 Qy', 'LineWidth', 1.5);
% plot(time_aligned, qz2, 'r-.', 'DisplayName', 'C2 Qz', 'LineWidth', 1.5);
% Add vertical dashed lines at time_cuts
for i = 1:length(time_cuts)
    xline(time_cuts(i), 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
end
xlabel('Time (s)');
ylabel('Quaternion Value');
xlim([13.8, 30.82]);
title('Quaternion Components');
legend('Location', 'best', 'NumColumns', 2);
grid on;

% Plot 3: Euler Angles
subplot(3, 1, 3);
plot(time_aligned, roll1, '-', 'Color', [0, 0.4470, 0.7410], 'LineWidth', 1.5, 'DisplayName', 'C1 Roll');
hold on;
plot(time_aligned, pitch1, '-', 'Color', [0.8500, 0.3250, 0.0980], 'LineWidth', 1.5, 'DisplayName', 'C1 Yaw');
plot(time_aligned, yaw1, '-', 'Color', [0.4940, 0.1840, 0.5560], 'LineWidth', 1.5, 'DisplayName', 'C1 Roll');
% plot(time_aligned, roll2, 'r-', 'LineWidth', 1.5, 'DisplayName', 'C2 Roll');
% plot(time_aligned, pitch2, 'r--', 'LineWidth', 1.5, 'DisplayName', 'C2 Pitch');
% plot(time_aligned, yaw2, 'r:', 'LineWidth', 1.5, 'DisplayName', 'C2 Yaw');
% Add vertical dashed lines at time_cuts
for i = 1:length(time_cuts)
    xline(time_cuts(i), 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
end
xlabel('Time (s)');
ylabel('Angle (degrees)');
xlim([13.8, 30.82]);
title('Euler Angles (Roll, Pitch, Yaw)');
legend('Location', 'best', 'NumColumns', 2);
grid on;

%% ========================================================================
% SEGMENT DATA BASED ON TIME CUTS
% =========================================================================

fprintf('Segmenting data into %d gait cycles...\n', num_segments);

% Initialize cell arrays to store segments (for current file visualization)
segments_force1 = cell(num_segments, 1);
segments_force2 = cell(num_segments, 1);
segments_qw1 = cell(num_segments, 1);
segments_qx1 = cell(num_segments, 1);
segments_qy1 = cell(num_segments, 1);
segments_qz1 = cell(num_segments, 1);
segments_qw2 = cell(num_segments, 1);
segments_qx2 = cell(num_segments, 1);
segments_qy2 = cell(num_segments, 1);
segments_qz2 = cell(num_segments, 1);
segments_roll1 = cell(num_segments, 1);
segments_pitch1 = cell(num_segments, 1);
segments_yaw1 = cell(num_segments, 1);
segments_roll2 = cell(num_segments, 1);
segments_pitch2 = cell(num_segments, 1);
segments_yaw2 = cell(num_segments, 1);
segments_time = cell(num_segments, 1);

% Initialize array to store segments for MAT file
new_segments = struct([]);

% Extract segments between time cuts
for i = 1:num_segments
    % Find indices for this segment
    start_time = time_cuts(i);
    end_time = time_cuts(i + 1);
    
    idx = find(time_aligned >= start_time & time_aligned < end_time);
    
    if isempty(idx)
        warning('Segment %d is empty (time: %.2f to %.2f)', i, start_time, end_time);
        continue;
    end
    
    % Calculate segment duration and normalize time to [0, 1]
    segment_duration = end_time - start_time;
    segments_time{i} = (time_aligned(idx) - start_time) / segment_duration; % Normalize to 0-1
    segments_force1{i} = force1(idx);
    segments_force2{i} = force2(idx);
    
    segments_qw1{i} = qw1(idx);
    segments_qx1{i} = qx1(idx);
    segments_qy1{i} = qy1(idx);
    segments_qz1{i} = qz1(idx);
    
    segments_qw2{i} = qw2(idx);
    segments_qx2{i} = qx2(idx);
    segments_qy2{i} = qy2(idx);
    segments_qz2{i} = qz2(idx);
    
    segments_roll1{i} = roll1(idx);
    segments_pitch1{i} = pitch1(idx);
    segments_yaw1{i} = yaw1(idx);
    
    segments_roll2{i} = roll2(idx);
    segments_pitch2{i} = pitch2(idx);
    segments_yaw2{i} = yaw2(idx);
    
    % Store in struct array for MAT file
    new_segments(i).time = segments_time{i};
    new_segments(i).force1 = segments_force1{i};
    new_segments(i).force2 = segments_force2{i};
    new_segments(i).qw1 = segments_qw1{i};
    new_segments(i).qx1 = segments_qx1{i};
    new_segments(i).qy1 = segments_qy1{i};
    new_segments(i).qz1 = segments_qz1{i};
    new_segments(i).qw2 = segments_qw2{i};
    new_segments(i).qx2 = segments_qx2{i};
    new_segments(i).qy2 = segments_qy2{i};
    new_segments(i).qz2 = segments_qz2{i};
    new_segments(i).roll1 = segments_roll1{i};
    new_segments(i).pitch1 = segments_pitch1{i};
    new_segments(i).yaw1 = segments_yaw1{i};
    new_segments(i).roll2 = segments_roll2{i};
    new_segments(i).pitch2 = segments_pitch2{i};
    new_segments(i).yaw2 = segments_yaw2{i};
    new_segments(i).source_file = filename;
    new_segments(i).segment_num = i;
    new_segments(i).original_start_time = start_time;
    new_segments(i).original_end_time = end_time;
    new_segments(i).duration = segment_duration;
    
    fprintf('  Segment %d: %.2f to %.2f s (%.2f s duration, %d samples) -> normalized to [0, 1]\n', ...
        i, start_time, end_time, segment_duration, length(idx));
end

%% ========================================================================
% SAVE/APPEND SEGMENTS TO MAT FILE
% =========================================================================

fprintf('\nSaving segments to MAT file...\n');

% Check if MAT file exists
if exist(segments_mat_file, 'file')
    % Load existing segments
    fprintf('Loading existing segments from: %s\n', segments_mat_file);
    loaded_data = load(segments_mat_file);
    all_segments = loaded_data.segments;
    previous_count = length(all_segments);
    
    % Append new segments
    all_segments = [all_segments; new_segments(:)];
    
    fprintf('??? Appended %d new segments to existing %d segments\n', ...
        length(new_segments), previous_count);
    fprintf('??? Total segments in database: %d\n', length(all_segments));
else
    % Create new file
    all_segments = new_segments(:);
    fprintf('??? Created new segments database with %d segments\n', length(all_segments));
end

% Save to MAT file
segments = all_segments;
save(segments_mat_file, 'segments', '-v7.3');
fprintf('??? Saved to: %s\n', segments_mat_file);

% Display summary of what's in the database
unique_files = unique({segments.source_file});
fprintf('\nDatabase summary:\n');
fprintf('  Total segments: %d\n', length(segments));
fprintf('  From %d different files:\n', length(unique_files));
for i = 1:length(unique_files)
    file_segments = sum(strcmp({segments.source_file}, unique_files{i}));
    fprintf('    - %s: %d segments\n', unique_files{i}, file_segments);
end


%% ========================================================================
% FIGURE 1.1: (Force as % Body Weight)
% =========================================================================

fprintf('Creating Figure 1.1: Full data visualization...\n');

% Define body weight in grams
body_weight_grams = 93000+17680;  % 73 kg = 73000 grams

% Convert forces to percentage of body weight
force1_percent = (force1*1.1827 / body_weight_grams) * 100;
force2_percent = (force2*1.1827 / body_weight_grams) * 100;

% Create binary signal: 0.25 when either force > 0, otherwise 0
combined_force_signal = zeros(size(force1_percent));
combined_force_signal((force1_percent > 5) | (force2_percent > 5)) = 25;

figure('Name', 'Full Crutch Data Analysis', 'Position', [100, 100, 1200, 900]);

% Plot 1: Forces (as % Body Weight)
subplot(3, 1, 1);
plot(time_aligned, force1_percent, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Crutch 1 (Right)');
hold on;
plot(time_aligned, force2_percent, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Crutch 2 (Left)');
% plot(time_aligned, combined_force_signal, 'k-', 'LineWidth', 2, 'DisplayName', 'Combined Activity (25% BW)');
for i = 1:length(time_cuts)
    xline(time_cuts(i), 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
end
xlabel('Time (s)');
ylabel('Force (% System + Body Weight)');
xlim([13.8, 30.82]);
title('Force Measurements (% Body Weight, BW = 110 kg)');
legend('Location', 'best');
grid on;

% Plot 2: Quaternions
subplot(3, 1, 2);
plot(time_aligned, qw1, '-', 'Color', [0, 0.4470, 0.7410],'DisplayName', 'C1 Qw', 'LineWidth', 1.5);
hold on;
plot(time_aligned, qx1, '-', 'Color', [0.8500, 0.3250, 0.0980],'DisplayName', 'C1 Qx', 'LineWidth', 1.5);
plot(time_aligned, qy1, '-', 'Color', [0.9290, 0.6940, 0.1250],'DisplayName', 'C1 Qy', 'LineWidth', 1.5);
plot(time_aligned, qz1, '-', 'Color', [0.4940, 0.1840, 0.5560],'DisplayName', 'C1 Qz', 'LineWidth', 1.5);
% Add vertical dashed lines at time_cuts
for i = 1:length(time_cuts)
    xline(time_cuts(i), 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
end
xlabel('Time (s)');
ylabel('Quaternion Value');
xlim([13.8, 30.82]);
title('Quaternion Components');
legend('Location', 'best', 'NumColumns', 2);
grid on;

% Plot 3: Euler Angles
subplot(3, 1, 3);
plot(time_aligned, roll1, '-', 'Color', [0, 0.4470, 0.7410],'LineWidth', 1.5, 'DisplayName', 'C1 Pitch');
hold on;
plot(time_aligned, pitch1, '-', 'Color',[0.8500, 0.3250, 0.0980], 'LineWidth', 1.5, 'DisplayName', 'C1 Yaw');
plot(time_aligned, yaw1, '-', 'Color', [0.4940, 0.1840, 0.5560],'LineWidth', 1.5, 'DisplayName', 'C1 Roll');
% Add vertical dashed lines at time_cuts
for i = 1:length(time_cuts)
    xline(time_cuts(i), 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
end
xlabel('Time (s)');
ylabel('Angle (degrees)');
xlim([13.8, 30.82]);
title('Euler Angles (Roll, Pitch, Yaw)');
legend('Location', 'best', 'NumColumns', 2);
grid on;


%% ========================================================================
% FIGURE 2: ALL SEGMENTS OVERLAID (FORCE) - CURRENT FILE ONLY
% =========================================================================
% 
% fprintf('\nCreating Figure 2: Force segments overlay (current file)...\n');
% 
% figure('Name', 'Force Segments Overlay', 'Position', [150, 150, 1200, 400]);
% 
% % Crutch 1 Force
% subplot(1, 2, 1);
% hold on;
% colors = winter(num_segments);  % Better color scheme without yellow
% for i = 1:num_segments
%     if ~isempty(segments_force1{i})
%         plot(segments_time{i}, segments_force1{i}, 'Color', colors(i, :), ...
%             'LineWidth', 1, 'DisplayName', sprintf('Seg %d', i));
%     end
% end
% xlabel('Gait Cycle (%)');
% ylabel('Force (units)');
% title('Crutch 1 - All Force Segments');
% xticks(0:0.2:1);
% xticklabels({'0%', '20%', '40%', '60%', '80%', '100%'});
% legend('Location', 'best');
% grid on;
% 
% % Crutch 2 Force
% subplot(1, 2, 2);
% hold on;
% for i = 1:num_segments
%     if ~isempty(segments_force2{i})
%         plot(segments_time{i}, segments_force2{i}, 'Color', colors(i, :), ...
%             'LineWidth', 1, 'DisplayName', sprintf('Seg %d', i));
%     end
% end
% xlabel('Gait Cycle (%)');
% ylabel('Force (units)');
% title('Crutch 2 - All Force Segments');
% xticks(0:0.2:1);
% xticklabels({'0%', '20%', '40%', '60%', '80%', '100%'});
% legend('Location', 'best');
% grid on;
% 
% %% ========================================================================
% % FIGURE 3: ALL SEGMENTS OVERLAID (QUATERNIONS) - CURRENT FILE ONLY
% % =========================================================================
% 
% fprintf('Creating Figure 3: Quaternion segments overlay (current file)...\n');
% 
% figure('Name', 'Quaternion Segments Overlay', 'Position', [200, 200, 1200, 900]);
% 
% quaternion_labels = {'Qw', 'Qx', 'Qy', 'Qz'};
% for q = 1:4
%     % Crutch 1
%     subplot(4, 2, (q-1)*2 + 1);
%     hold on;
%     for i = 1:num_segments
%         switch q
%             case 1, data_seg = segments_qw1{i};
%             case 2, data_seg = segments_qx1{i};
%             case 3, data_seg = segments_qy1{i};
%             case 4, data_seg = segments_qz1{i};
%         end
%         if ~isempty(data_seg)
%             plot(segments_time{i}, data_seg, 'Color', colors(i, :), ...
%                 'LineWidth', 1, 'DisplayName', sprintf('Seg %d', i));
%         end
%     end
%     xlabel('Gait Cycle (%)');
%     ylabel(quaternion_labels{q});
%     title(sprintf('Crutch 1 - %s Segments', quaternion_labels{q}));
%     xticks(0:0.2:1);
%     xticklabels({'0%', '20%', '40%', '60%', '80%', '100%'});
%     if q == 1
%         legend('Location', 'best', 'NumColumns', 2);
%     end
%     grid on;
%     
%     % Crutch 2
%     subplot(4, 2, (q-1)*2 + 2);
%     hold on;
%     for i = 1:num_segments
%         switch q
%             case 1, data_seg = segments_qw2{i};
%             case 2, data_seg = segments_qx2{i};
%             case 3, data_seg = segments_qy2{i};
%             case 4, data_seg = segments_qz2{i};
%         end
%         if ~isempty(data_seg)
%             plot(segments_time{i}, data_seg, 'Color', colors(i, :), ...
%                 'LineWidth', 1, 'DisplayName', sprintf('Seg %d', i));
%         end
%     end
%     xlabel('Gait Cycle (%)');
%     ylabel(quaternion_labels{q});
%     title(sprintf('Crutch 2 - %s Segments', quaternion_labels{q}));
%     xticks(0:0.2:1);
%     xticklabels({'0%', '20%', '40%', '60%', '80%', '100%'});
%     if q == 1
%         legend('Location', 'best', 'NumColumns', 2);
%     end
%     grid on;
% end
% 
% %% ========================================================================
% % FIGURE 4: ALL SEGMENTS OVERLAID (EULER ANGLES) - CURRENT FILE ONLY
% % =========================================================================
% 
% fprintf('Creating Figure 4: Euler angle segments overlay (current file)...\n');
% 
% figure('Name', 'Euler Angle Segments Overlay', 'Position', [250, 250, 1200, 900]);
% 
% euler_labels = {'Roll', 'Pitch', 'Yaw'};
% for e = 1:3
%     % Crutch 1
%     subplot(3, 2, (e-1)*2 + 1);
%     hold on;
%     for i = 1:num_segments
%         switch e
%             case 1, data_seg = segments_roll1{i};
%             case 2, data_seg = segments_pitch1{i};
%             case 3, data_seg = segments_yaw1{i};
%         end
%         if ~isempty(data_seg)
%             plot(segments_time{i}, data_seg, 'Color', colors(i, :), ...
%                 'LineWidth', 1, 'DisplayName', sprintf('Seg %d', i));
%         end
%     end
%     xlabel('Gait Cycle (%)');
%     ylabel(sprintf('%s (deg)', euler_labels{e}));
%     title(sprintf('Crutch 1 - %s Segments', euler_labels{e}));
%     xticks(0:0.2:1);
%     xticklabels({'0%', '20%', '40%', '60%', '80%', '100%'});
%     if e == 1
%         legend('Location', 'best', 'NumColumns', 2);
%     end
%     grid on;
%     
%     % Crutch 2
%     subplot(3, 2, (e-1)*2 + 2);
%     hold on;
%     for i = 1:num_segments
%         switch e
%             case 1, data_seg = segments_roll2{i};
%             case 2, data_seg = segments_pitch2{i};
%             case 3, data_seg = segments_yaw2{i};
%         end
%         if ~isempty(data_seg)
%             plot(segments_time{i}, data_seg, 'Color', colors(i, :), ...
%                 'LineWidth', 1, 'DisplayName', sprintf('Seg %d', i));
%         end
%     end
%     xlabel('Gait Cycle (%)');
%     ylabel(sprintf('%s (deg)', euler_labels{e}));
%     title(sprintf('Crutch 2 - %s Segments', euler_labels{e}));
%     xticks(0:0.2:1);
%     xticklabels({'0%', '20%', '40%', '60%', '80%', '100%'});
%     if e == 1
%         legend('Location', 'best', 'NumColumns', 2);
%     end
%     grid on;
% end
% 
% %% ========================================================================
% % COMPUTE MEAN AND STANDARD DEVIATION - CURRENT FILE ONLY
% % =========================================================================
% 
% fprintf('Computing mean and standard deviation across segments (current file)...\n');
% 
% % All segments are now normalized to [0, 1]
% % Create common time base from 0 to 1 (0% to 100% of gait cycle)
% common_time = linspace(0, 1, 200); % 200 points
% 
% % Initialize matrices for resampled data
% force1_matrix = zeros(length(common_time), num_segments);
% force2_matrix = zeros(length(common_time), num_segments);
% roll1_matrix = zeros(length(common_time), num_segments);
% pitch1_matrix = zeros(length(common_time), num_segments);
% yaw1_matrix = zeros(length(common_time), num_segments);
% roll2_matrix = zeros(length(common_time), num_segments);
% pitch2_matrix = zeros(length(common_time), num_segments);
% yaw2_matrix = zeros(length(common_time), num_segments);
% qw1_matrix = zeros(length(common_time), num_segments);
% qx1_matrix = zeros(length(common_time), num_segments);
% qy1_matrix = zeros(length(common_time), num_segments);
% qz1_matrix = zeros(length(common_time), num_segments);
% qw2_matrix = zeros(length(common_time), num_segments);
% qx2_matrix = zeros(length(common_time), num_segments);
% qy2_matrix = zeros(length(common_time), num_segments);
% qz2_matrix = zeros(length(common_time), num_segments);
% 
% % Resample each segment to common time base
% for i = 1:num_segments
%     if ~isempty(segments_time{i})
%         force1_matrix(:, i) = interp1(segments_time{i}, segments_force1{i}, common_time, 'linear', 'extrap');
%         force2_matrix(:, i) = interp1(segments_time{i}, segments_force2{i}, common_time, 'linear', 'extrap');
%         
%         roll1_matrix(:, i) = interp1(segments_time{i}, segments_roll1{i}, common_time, 'linear', 'extrap');
%         pitch1_matrix(:, i) = interp1(segments_time{i}, segments_pitch1{i}, common_time, 'linear', 'extrap');
%         yaw1_matrix(:, i) = interp1(segments_time{i}, segments_yaw1{i}, common_time, 'linear', 'extrap');
%         
%         roll2_matrix(:, i) = interp1(segments_time{i}, segments_roll2{i}, common_time, 'linear', 'extrap');
%         pitch2_matrix(:, i) = interp1(segments_time{i}, segments_pitch2{i}, common_time, 'linear', 'extrap');
%         yaw2_matrix(:, i) = interp1(segments_time{i}, segments_yaw2{i}, common_time, 'linear', 'extrap');
%         
%         qw1_matrix(:, i) = interp1(segments_time{i}, segments_qw1{i}, common_time, 'linear', 'extrap');
%         qx1_matrix(:, i) = interp1(segments_time{i}, segments_qx1{i}, common_time, 'linear', 'extrap');
%         qy1_matrix(:, i) = interp1(segments_time{i}, segments_qy1{i}, common_time, 'linear', 'extrap');
%         qz1_matrix(:, i) = interp1(segments_time{i}, segments_qz1{i}, common_time, 'linear', 'extrap');
%         
%         qw2_matrix(:, i) = interp1(segments_time{i}, segments_qw2{i}, common_time, 'linear', 'extrap');
%         qx2_matrix(:, i) = interp1(segments_time{i}, segments_qx2{i}, common_time, 'linear', 'extrap');
%         qy2_matrix(:, i) = interp1(segments_time{i}, segments_qy2{i}, common_time, 'linear', 'extrap');
%         qz2_matrix(:, i) = interp1(segments_time{i}, segments_qz2{i}, common_time, 'linear', 'extrap');
%     end
% end
% 
% % Compute mean and std
% force1_mean = mean(force1_matrix, 2);
% force1_std = std(force1_matrix, 0, 2);
% force2_mean = mean(force2_matrix, 2);
% force2_std = std(force2_matrix, 0, 2);
% 
% roll1_mean = mean(roll1_matrix, 2);
% roll1_std = std(roll1_matrix, 0, 2);
% pitch1_mean = mean(pitch1_matrix, 2);
% pitch1_std = std(pitch1_matrix, 0, 2);
% yaw1_mean = mean(yaw1_matrix, 2);
% yaw1_std = std(yaw1_matrix, 0, 2);
% 
% roll2_mean = mean(roll2_matrix, 2);
% roll2_std = std(roll2_matrix, 0, 2);
% pitch2_mean = mean(pitch2_matrix, 2);
% pitch2_std = std(pitch2_matrix, 0, 2);
% yaw2_mean = mean(yaw2_matrix, 2);
% yaw2_std = std(yaw2_matrix, 0, 2);
% 
% qw1_mean = mean(qw1_matrix, 2);
% qw1_std = std(qw1_matrix, 0, 2);
% qx1_mean = mean(qx1_matrix, 2);
% qx1_std = std(qx1_matrix, 0, 2);
% qy1_mean = mean(qy1_matrix, 2);
% qy1_std = std(qy1_matrix, 0, 2);
% qz1_mean = mean(qz1_matrix, 2);
% qz1_std = std(qz1_matrix, 0, 2);
% 
% qw2_mean = mean(qw2_matrix, 2);
% qw2_std = std(qw2_matrix, 0, 2);
% qx2_mean = mean(qx2_matrix, 2);
% qx2_std = std(qx2_matrix, 0, 2);
% qy2_mean = mean(qy2_matrix, 2);
% qy2_std = std(qy2_matrix, 0, 2);
% qz2_mean = mean(qz2_matrix, 2);
% qz2_std = std(qz2_matrix, 0, 2);
% 
% %% ========================================================================
% % FIGURE 5: MEAN +/- 2*STD (SEPARATED BY CRUTCH) - CURRENT FILE ONLY
% % =========================================================================
% 
% fprintf('Creating Figure 5: Mean with 2*STD shaded regions (current file)...\n');
% 
% figure('Name', sprintf('Mean Signals with 2*STD - %s', filename), 'Position', [300, 300, 1400, 900]);
% 
% % LEFT COLUMN - CRUTCH 1 (COM10)
% % Plot 1: Crutch 1 Force
% subplot(3, 2, 1);
% hold on;
% plot_shaded(common_time, force1_mean, 2*force1_std, [0, 0.4470, 0.7410], 'Force');
% xlabel('Gait Cycle (%)');
% ylabel('Force (units)');
% title('Crutch 1 (COM10) - Mean Force +/- 2*STD');
% xticks(0:0.2:1);
% xticklabels({'0%', '20%', '40%', '60%', '80%', '100%'});
% legend('Location', 'best');
% grid on;
% 
% % Plot 3: Crutch 1 Quaternions
% subplot(3, 2, 3);
% plot_shaded(common_time, qw1_mean, 2*qw1_std, [0, 0.4470, 0.7410], 'Qw');
% hold on;
% plot_shaded(common_time, qx1_mean, 2*qx1_std, [0.8500, 0.3250, 0.0980], 'Qx');
% plot_shaded(common_time, qy1_mean, 2*qy1_std, [0.9290, 0.6940, 0.1250], 'Qy');
% plot_shaded(common_time, qz1_mean, 2*qz1_std, [0.4940, 0.1840, 0.5560], 'Qz');
% xlabel('Gait Cycle (%)');
% ylabel('Quaternion Value');
% title('Crutch 1 (COM10) - Mean Quaternions +/- 2*STD');
% xticks(0:0.2:1);
% xticklabels({'0%', '20%', '40%', '60%', '80%', '100%'});
% legend('Location', 'best');
% grid on;
% 
% % Plot 5: Crutch 1 Euler Angles
% subplot(3, 2, 5);
% plot_shaded(common_time, roll1_mean, 2*roll1_std, [0, 0.4470, 0.7410], 'Roll');
% hold on;
% plot_shaded(common_time, pitch1_mean, 2*pitch1_std, [0.8500, 0.3250, 0.0980], 'Pitch');
% plot_shaded(common_time, yaw1_mean, 2*yaw1_std, [0.4940, 0.1840, 0.5560], 'Yaw');
% xlabel('Gait Cycle (%)');
% ylabel('Angle (degrees)');
% title('Crutch 1 (COM10) - Mean Euler Angles +/- 2*STD');
% xticks(0:0.2:1);
% xticklabels({'0%', '20%', '40%', '60%', '80%', '100%'});
% legend('Location', 'best');
% grid on;
% 
% % RIGHT COLUMN - CRUTCH 2 (COM12)
% % Plot 2: Crutch 2 Force
% subplot(3, 2, 2);
% hold on;
% plot_shaded(common_time, force2_mean, 2*force2_std, [0.6350, 0.0780, 0.1840], 'Force');
% xlabel('Gait Cycle (%)');
% ylabel('Force (units)');
% title('Crutch 2 (COM12) - Mean Force +/- 2*STD');
% xticks(0:0.2:1);
% xticklabels({'0%', '20%', '40%', '60%', '80%', '100%'});
% legend('Location', 'best');
% grid on;
% 
% % Plot 4: Crutch 2 Quaternions
% subplot(3, 2, 4);
% plot_shaded(common_time, qw2_mean, 2*qw2_std, [0.6350, 0.0780, 0.1840], 'Qw');
% hold on;
% plot_shaded(common_time, qx2_mean, 2*qx2_std, [0.8500, 0.3250, 0.0980], 'Qx');
% plot_shaded(common_time, qy2_mean, 2*qy2_std, [0.9290, 0.6940, 0.1250], 'Qy');
% plot_shaded(common_time, qz2_mean, 2*qz2_std, [0.4940, 0.1840, 0.5560], 'Qz');
% xlabel('Gait Cycle (%)');
% ylabel('Quaternion Value');
% title('Crutch 2 (COM12) - Mean Quaternions +/- 2*STD');
% xticks(0:0.2:1);
% xticklabels({'0%', '20%', '40%', '60%', '80%', '100%'});
% legend('Location', 'best');
% grid on;
% 
% % Plot 6: Crutch 2 Euler Angles
% subplot(3, 2, 6);
% plot_shaded(common_time, roll2_mean, 2*roll2_std, [0.6350, 0.0780, 0.1840], 'Roll');
% hold on;
% plot_shaded(common_time, pitch2_mean, 2*pitch2_std, [0.8500, 0.3250, 0.0980], 'Pitch');
% plot_shaded(common_time, yaw2_mean, 2*yaw2_std, [0.4940, 0.1840, 0.5560], 'Yaw');
% xlabel('Gait Cycle (%)');
% ylabel('Angle (degrees)');
% title('Crutch 2 (COM12) - Mean Euler Angles +/- 2*STD');
% xticks(0:0.2:1);
% xticklabels({'0%', '20%', '40%', '60%', '80%', '100%'});
% legend('Location', 'best');
% grid on;
% 
% %% ========================================================================
% % SUMMARY
% % =========================================================================
% 
% fprintf('\n' + string(repmat('=', 1, 70)) + '\n');
% fprintf('ANALYSIS COMPLETE!\n');
% fprintf(string(repmat('=', 1, 70)) + '\n');
% fprintf('Current file: %s\n', filename);
% fprintf('  - Extracted %d segments\n', num_segments);
% fprintf('  - Saved to: %s\n', segments_mat_file);
% fprintf('\nDatabase summary:\n');
% fprintf('  - Total segments in database: %d\n', length(segments));
% fprintf('  - From %d different files\n', length(unique_files));
% fprintf('\nGenerated 5 figures:\n');
% fprintf('  1. Full data (Force, Quaternions, Euler)\n');
% fprintf('  2. Force segments overlay (current file)\n');
% fprintf('  3. Quaternion segments overlay (current file)\n');
% fprintf('  4. Euler angle segments overlay (current file)\n');
% fprintf('  5. Mean signals with 2*STD (current file)\n');
% fprintf('\nTo analyze ALL accumulated segments, run: analyze_all_segments.m\n');
% fprintf(string(repmat('=', 1, 70)) + '\n\n');

%% ========================================================================
% HELPER FUNCTIONS
% =========================================================================

%% Helper Function: Quaternion to Euler Conversion
function [roll, pitch, yaw] = quat2euler(qw, qx, qy, qz)
    % Convert quaternions to Euler angles (Roll, Pitch, Yaw) in degrees
    % Using ZYX convention (Yaw-Pitch-Roll)
    
    % Roll (x-axis rotation)
    sinr_cosp = 2 * (qw .* qx + qy .* qz);
    cosr_cosp = 1 - 2 * (qx .* qx + qy .* qy);
    roll = atan2(sinr_cosp, cosr_cosp) * 180 / pi;
    
    % Pitch (y-axis rotation)
    sinp = 2 * (qw .* qy - qz .* qx);
    pitch = zeros(size(sinp));
    % Handle gimbal lock
    idx = abs(sinp) >= 1;
    pitch(idx) = sign(sinp(idx)) * 90; % Use 90 degrees if out of range
    pitch(~idx) = asin(sinp(~idx)) * 180 / pi;
    
    % Yaw (z-axis rotation)
    siny_cosp = 2 * (qw .* qz + qx .* qy);
    cosy_cosp = 1 - 2 * (qy .* qy + qz .* qz);
    yaw = atan2(siny_cosp, cosy_cosp) * 180 / pi;
end

%% Helper Function: Plot with Shaded Standard Deviation (FIXED VERSION)
function plot_shaded(x, y_mean, y_std, color, label)
    % Plot mean line with shaded standard deviation region
    
    % Calculate upper and lower bounds
    y_upper = y_mean + y_std;
    y_lower = y_mean - y_std;
    
    % Convert color to RGB if needed
    if ischar(color)
        % Convert color letter to RGB
        switch color
            case 'b', rgb = [0, 0, 1];
            case 'r', rgb = [1, 0, 0];
            case 'g', rgb = [0, 1, 0];
            case 'y', rgb = [1, 1, 0];
            case 'm', rgb = [1, 0, 1];
            case 'c', rgb = [0, 1, 1];
            case 'k', rgb = [0, 0, 0];
            otherwise, rgb = [0.5, 0.5, 0.5];
        end
    else
        rgb = color;
    end
    
    % Save current hold state and enable hold
    wasHeld = ishold;
    hold on;
    
    % Plot shaded area (this needs to go first, behind the line)
    fill([x(:)', fliplr(x(:)')], [y_upper(:)', fliplr(y_lower(:)')], rgb, ...
        'FaceAlpha', 0.2, 'EdgeColor', 'none', 'HandleVisibility', 'off');
    
    % Plot mean line on top
    plot(x, y_mean, 'Color', rgb, 'LineWidth', 2, 'DisplayName', label);
    
    % Restore original hold state
    if ~wasHeld
        hold off;
    end
end
