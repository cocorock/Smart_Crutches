% MATLAB Script to Analyze Right Crutch (COM10) Data from ALL Accumulated Segments
% Loads segments from MAT file and computes statistics for right crutch only
% Author: Created for Crutch HMI System
% Date: 2025

clear all;
close all;
clc;

%% ========================================================================
% CONFIGURATION
% =========================================================================

% MAT file containing all segments
segments_mat_file = 'all_gait_segments.mat';

% Common time base for resampling (0 to 1, representing 0% to 100%)
common_time = linspace(0, 1, 200);

%% ========================================================================
% LOAD ALL SEGMENTS
% =========================================================================

fprintf('Loading segments database from: %s\n', segments_mat_file);

if ~exist(segments_mat_file, 'file')
    error('Segments database file not found: %s\nPlease run process_dual_crutch_data_with_database.m first.', ...
        segments_mat_file);
end

% Load segments
loaded_data = load(segments_mat_file);
segments = loaded_data.segments;

num_segments = length(segments);
fprintf('✓ Loaded %d segments\n', num_segments);

% Display database summary
unique_files = unique({segments.source_file});
fprintf('\nDatabase contains segments from %d files:\n', length(unique_files));
for i = 1:length(unique_files)
    file_segments = sum(strcmp({segments.source_file}, unique_files{i}));
    fprintf('  %d. %s: %d segments\n', i, unique_files{i}, file_segments);
end

%% ========================================================================
% RESAMPLE ALL SEGMENTS TO COMMON TIME BASE - RIGHT CRUTCH ONLY
% =========================================================================

fprintf('\nResampling all right crutch segments to common time base...\n');

% Initialize matrices for right crutch (Crutch 1 / COM10)
force1_matrix = zeros(length(common_time), num_segments);

qw1_matrix = zeros(length(common_time), num_segments);
qx1_matrix = zeros(length(common_time), num_segments);
qy1_matrix = zeros(length(common_time), num_segments);
qz1_matrix = zeros(length(common_time), num_segments);

roll1_matrix = zeros(length(common_time), num_segments);
pitch1_matrix = zeros(length(common_time), num_segments);
yaw1_matrix = zeros(length(common_time), num_segments);

% Resample each segment
for i = 1:num_segments
    seg = segments(i);
    
    % Resample to common time base - Right crutch only
    force1_matrix(:, i) = interp1(seg.time, seg.force1, common_time, 'linear', 'extrap');
    
    qw1_matrix(:, i) = interp1(seg.time, seg.qw1, common_time, 'linear', 'extrap');
    qx1_matrix(:, i) = interp1(seg.time, seg.qx1, common_time, 'linear', 'extrap');
    qy1_matrix(:, i) = interp1(seg.time, seg.qy1, common_time, 'linear', 'extrap');
    qz1_matrix(:, i) = interp1(seg.time, seg.qz1, common_time, 'linear', 'extrap');
    
    roll1_matrix(:, i) = interp1(seg.time, seg.roll1, common_time, 'linear', 'extrap');
    pitch1_matrix(:, i) = interp1(seg.time, seg.pitch1, common_time, 'linear', 'extrap');
    yaw1_matrix(:, i) = interp1(seg.time, seg.yaw1, common_time, 'linear', 'extrap');
end

fprintf('✓ Resampled %d segments to common time base\n', num_segments);

%% ========================================================================
% COMPUTE STATISTICS - RIGHT CRUTCH
% =========================================================================

fprintf('Computing mean and standard deviation across all right crutch segments...\n');

% Force statistics
force1_mean = mean(force1_matrix, 2);
force1_std = std(force1_matrix, 0, 2);

% Quaternion statistics
qw1_mean = mean(qw1_matrix, 2);
qw1_std = std(qw1_matrix, 0, 2);
qx1_mean = mean(qx1_matrix, 2);
qx1_std = std(qx1_matrix, 0, 2);
qy1_mean = mean(qy1_matrix, 2);
qy1_std = std(qy1_matrix, 0, 2);
qz1_mean = mean(qz1_matrix, 2);
qz1_std = std(qz1_matrix, 0, 2);

% Euler angle statistics
roll1_mean = mean(roll1_matrix, 2);
roll1_std = std(roll1_matrix, 0, 2);
pitch1_mean = mean(pitch1_matrix, 2);
pitch1_std = std(pitch1_matrix, 0, 2);
yaw1_mean = mean(yaw1_matrix, 2);
yaw1_std = std(yaw1_matrix, 0, 2);

fprintf('✓ Statistics computed\n');

%% ========================================================================
% FIGURE 1: ALL SEGMENTS OVERLAID - FORCE
% =========================================================================

fprintf('\nCreating visualizations...\n');
fprintf('Figure 1: All force segments overlaid...\n');

figure('Name', sprintf('Right Crutch - All Force Segments (n=%d)', num_segments), ...
    'Position', [100, 100, 800, 600]);

hold on;
colors = parula(num_segments);
for i = 1:num_segments
    plot(common_time, force1_matrix(:, i), 'Color', [colors(i,:), 0.3], 'LineWidth', 0.5);
end
xlabel('Gait Cycle (%)');
ylabel('Force (units)');
title(sprintf('Right Crutch - All Force Segments (n=%d)', num_segments));
xticks(0:0.2:1);
xticklabels({'0%', '20%', '40%', '60%', '80%', '100%'});
grid on;

%% ========================================================================
% FIGURE 2: ALL SEGMENTS OVERLAID - QUATERNIONS
% =========================================================================

fprintf('Figure 2: All quaternion segments overlaid...\n');

figure('Name', sprintf('Right Crutch - All Quaternion Segments (n=%d)', num_segments), ...
    'Position', [150, 150, 800, 900]);

quaternion_labels = {'Qw', 'Qx', 'Qy', 'Qz'};
quat_matrices = {qw1_matrix, qx1_matrix, qy1_matrix, qz1_matrix};

for q = 1:4
    subplot(4, 1, q);
    hold on;
    for i = 1:num_segments
        plot(common_time, quat_matrices{q}(:, i), 'Color', [colors(i,:), 0.3], 'LineWidth', 0.5);
    end
    xlabel('Gait Cycle (%)');
    ylabel(quaternion_labels{q});
    title(sprintf('Right Crutch - %s (n=%d)', quaternion_labels{q}, num_segments));
    xticks(0:0.2:1);
    xticklabels({'0%', '20%', '40%', '60%', '80%', '100%'});
    grid on;
end

%% ========================================================================
% FIGURE 3: ALL SEGMENTS OVERLAID - EULER ANGLES
% =========================================================================

fprintf('Figure 3: All Euler angle segments overlaid...\n');

figure('Name', sprintf('Right Crutch - All Euler Angle Segments (n=%d)', num_segments), ...
    'Position', [200, 200, 800, 900]);

euler_labels = {'Roll', 'Pitch', 'Yaw'};
euler_matrices = {roll1_matrix, pitch1_matrix, yaw1_matrix};

for e = 1:3
    subplot(3, 1, e);
    hold on;
    for i = 1:num_segments
        plot(common_time, euler_matrices{e}(:, i), 'Color', [colors(i,:), 0.3], 'LineWidth', 0.5);
    end
    xlabel('Gait Cycle (%)');
    ylabel(sprintf('%s (deg)', euler_labels{e}));
    title(sprintf('Right Crutch - %s (n=%d)', euler_labels{e}, num_segments));
    xticks(0:0.2:1);
    xticklabels({'0%', '20%', '40%', '60%', '80%', '100%'});
    grid on;
end

%% ========================================================================
% FIGURE 4: MEAN WITH 2*STD - COMPREHENSIVE VIEW
% =========================================================================

fprintf('Figure 4: Mean with 2*STD (comprehensive view)...\n');

figure('Name', sprintf('Right Crutch - Mean Signals with 2*STD (n=%d)', num_segments), ...
    'Position', [250, 250, 800, 900]);

% Plot 1: Force
subplot(3, 1, 1);
hold on;
plot_shaded(common_time, force1_mean, 2*force1_std, [0, 0.4470, 0.7410], 'Force');
xlabel('Gait Cycle (%)');
ylabel('Force (units)');
title(sprintf('Right Crutch - Mean Force ± 2σ (n=%d)', num_segments));
xticks(0:0.2:1);
xticklabels({'0%', '20%', '40%', '60%', '80%', '100%'});
legend('Location', 'best');
grid on;

% Plot 2: Quaternions
subplot(3, 1, 2);
plot_shaded(common_time, qw1_mean, 2*qw1_std, [0, 0.4470, 0.7410], 'Qw');
hold on;
plot_shaded(common_time, qx1_mean, 2*qx1_std, [0.8500, 0.3250, 0.0980], 'Qx');
plot_shaded(common_time, qy1_mean, 2*qy1_std, [0.9290, 0.6940, 0.1250], 'Qy');
plot_shaded(common_time, qz1_mean, 2*qz1_std, [0.4940, 0.1840, 0.5560], 'Qz');
xlabel('Gait Cycle (%)');
ylabel('Quaternion Value');
title(sprintf('Right Crutch - Mean Quaternions ± 2σ (n=%d)', num_segments));
xticks(0:0.2:1);
xticklabels({'0%', '20%', '40%', '60%', '80%', '100%'});
legend('Location', 'best');
grid on;

% Plot 3: Euler Angles
subplot(3, 1, 3);
plot_shaded(common_time, roll1_mean, 2*roll1_std, [0, 0.4470, 0.7410], 'Roll');
hold on;
plot_shaded(common_time, pitch1_mean, 2*pitch1_std, [0.8500, 0.3250, 0.0980], 'Pitch');
plot_shaded(common_time, yaw1_mean, 2*yaw1_std, [0.4940, 0.1840, 0.5560], 'Yaw');
xlabel('Gait Cycle (%)');
ylabel('Angle (degrees)');
title(sprintf('Right Crutch - Mean Euler Angles ± 2σ (n=%d)', num_segments));
xticks(0:0.2:1);
xticklabels({'0%', '20%', '40%', '60%', '80%', '100%'});
legend('Location', 'best');
grid on;

%% ========================================================================
% SUMMARY
% =========================================================================

fprintf('\n' + string(repmat('=', 1, 70)) + '\n');
fprintf('ANALYSIS OF RIGHT CRUTCH - ALL ACCUMULATED SEGMENTS COMPLETE!\n');
fprintf(string(repmat('=', 1, 70)) + '\n');
fprintf('Database: %s\n', segments_mat_file);
fprintf('Total segments analyzed: %d\n', num_segments);
fprintf('From %d different files:\n', length(unique_files));
for i = 1:length(unique_files)
    file_segments = sum(strcmp({segments.source_file}, unique_files{i}));
    fprintf('  - %s: %d segments\n', unique_files{i}, file_segments);
end
fprintf('\nGenerated 4 figures:\n');
fprintf('  1. Right crutch - All force segments overlaid\n');
fprintf('  2. Right crutch - All quaternion segments overlaid\n');
fprintf('  3. Right crutch - All Euler angle segments overlaid\n');
fprintf('  4. Right crutch - Mean signals with 2*STD (comprehensive view)\n');
fprintf(string(repmat('=', 1, 70)) + '\n\n');

%% ========================================================================
% HELPER FUNCTION
% =========================================================================

%% Helper Function: Plot with Shaded Standard Deviation
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
