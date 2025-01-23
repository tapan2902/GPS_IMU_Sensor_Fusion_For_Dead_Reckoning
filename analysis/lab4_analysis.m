function [magX_final, magY_corrected, offsetX_hard_iron, offsetY_hard_iron, scalingFactor] = calc_circles_magnetometer_calib(magX_data, magY_data)
    % Function to perform and plot magnetometer calibration
    % Ensure the input arrays are of the same length
    if length(magX_data) ~= length(magY_data)
        error('Input arrays magX_data and magY_data must have the same length');
    end
    % Find the maximum and minimum values for each axis
    maxX = max(magX_data);
    minX = min(magX_data);
    maxY = max(magY_data);
    minY = min(magY_data);
    % Calculate the offset for each axis as (max + min) / 2
    offsetX_hard_iron = (maxX + minX) / 2;
    offsetY_hard_iron = (maxY + minY) / 2;
    % Subtract the hard-iron offset from the magnetometer readings
    magX_corrected = magX_data - offsetX_hard_iron;
    magY_corrected = magY_data - offsetY_hard_iron;
    % Find the range of each axis after hard-iron correction
    rangeX = max(magX_corrected) - min(magX_corrected);
    rangeY = max(magY_corrected) - min(magY_corrected);
    % Scale the axes to equalize the ranges
    scalingFactor = sqrt(rangeX / rangeY);  
    % Apply the scaling factors to the X-axis
    magX_final = magX_corrected / scalingFactor; % Adjust only X-axis to match Y-axis range

    figure;
    plot(magX_data, magY_data, 'b.','DisplayName','Before Hard-Iron and Soft-Iron Calibration');
    hold on;
    plot(magX_final, magY_corrected, 'r.','DisplayName','After Hard-Iron and Soft-Iron Calibration');
    line(xlim, [0 0], 'Color', 'k', 'LineWidth', 1.5); % Highlight Y-axis
    line([0 0], ylim, 'Color', 'k', 'LineWidth', 1.5); % Highlight X-axis
    xlabel('Magnetometer X (mG)');
    ylabel('Magnetometer Y (mG)');
    title('Magnetometer Before Calibration And After Calibration');
    legend('Before Calibration','After Calibration');
    grid on;
    hold off;   

end

function plotMagnetometerTimeSeries(driving_imu_timestamps,magX_data, magY_data, magX_calibrated, magY_calibrated)
    %Calculate the magnitude of the magnetometer data before correction
    magnitude_before = sqrt(magX_data.^2 + magY_data.^2);
    %Calculate the magnitude of the magnetometer data after correction
    magnitude_after = sqrt(magX_calibrated.^2 + magY_calibrated.^2);
    %Plot the time series of magnetometer data before and after calibration
    figure;
    plot(driving_imu_timestamps, magnitude_before, 'r', 'LineWidth', 1.5, 'DisplayName', 'Before Calibration');
    hold on;
    plot(driving_imu_timestamps, magnitude_after, 'b', 'LineWidth', 1.5, 'DisplayName', 'After Calibration');
    xlabel('Time (s)');
    ylabel('Magnetometer Magnitude (mG)');
    title('Driving Magnetometer Time Series: Before and After Calibration');
    legend('show');
    grid on;
    hold off;
end

function [driving_yaw_calibrated,driving_magX_corrected,driving_magY_corrected] = process_driving_magnetometer_data(magX_raw, magY_raw, offsetX_hard_iron, offsetY_hard_iron, scalingFactor, sample_rate)
    % Ensure the input arrays are of the same length
    if length(magX_raw) ~= length(magY_raw)
        error('Input arrays magX_raw and magY_raw must have the same length');
    end
    %Apply hard-iron correction
    driving_magX_corrected = magX_raw - offsetX_hard_iron;
    driving_magY_corrected = magY_raw - offsetY_hard_iron;
    %Apply soft-iron correction (scaling)
    driving_magX_corrected = driving_magX_corrected / scalingFactor;
    %Compute raw and calibrated yaw angles
    yaw_raw = atan2(magY_raw, magX_raw);  % Raw yaw angle in radians
    driving_yaw_calibrated = atan2(driving_magY_corrected, driving_magX_corrected);  % Calibrated yaw angle in radians
    %Create time vector based on the sample rate
    num_samples = length(magX_raw);
    time_vector = (0:num_samples-1) / sample_rate;
    %Plot raw yaw vs calibrated yaw
    figure;
    plot(time_vector, yaw_raw, 'r', 'DisplayName', 'Raw Yaw');
    hold on;
    plot(time_vector, driving_yaw_calibrated, 'b', 'DisplayName', 'Calibrated Yaw');
    xlabel('Time (s)');
    ylabel('Yaw Angle (radians)');
    title('Driving Data Yaw Angle: Raw vs Calibrated (Radians)');
    legend('show');
    grid on;
    hold off;
end

function [yaw_angle] = integrateYawRate(angular_velocity_z, time_vector)
    % Ensure the input arrays have the same length
    if length(angular_velocity_z) ~= length(time_vector)
        error('angular_velocity_z and time_vector must have the same length');
    end
    % Perform cumulative trapezoidal integration to get the yaw angle
    yaw_angle = cumtrapz(time_vector, angular_velocity_z);
    figure;
    plot(time_vector, yaw_angle, 'b', 'DisplayName', 'Integrated Yaw Angle');
    xlabel('Time (s)');
    ylabel('Yaw Angle (radians)');
    title('Yaw Angle from Gyroscope (Integrated Yaw Rate)');
    legend('show');
    grid on;
end

function plotYawAngles(driving_yaw_calibrated, driving_yaw_angle, driving_timestamp_vector)
    % Ensure the input arrays have the same length
    if length(driving_yaw_calibrated) ~= length(driving_yaw_angle) || length(driving_yaw_calibrated) ~= length(driving_timestamp_vector)
        error('Input arrays must have the same length');
    end
    figure;
    plot(driving_timestamp_vector, driving_yaw_calibrated, 'r', 'DisplayName', 'Magnetometer Yaw');
    hold on;
    plot(driving_timestamp_vector, driving_yaw_angle, 'b', 'DisplayName', 'Gyroscope Yaw (Integrated)');
    xlabel('Time (s)');
    ylabel('Yaw Angle (radians)');
    title('Magnetometer Yaw vs Yaw Integral from Gyro');
    legend('show');
    grid on;
    hold off;
end

function yaw_combined = complementaryFilterWithPlot(magnetometer_yaw, gyroscope_yaw, timestamp_vector, alpha_gyro, alpha_mag)
    % Initialize the array for the combined yaw angle
    yaw_combined = zeros(1, length(magnetometer_yaw));
    yaw_gyro_highpass = zeros(1, length(magnetometer_yaw));  % To store the high-pass filtered gyroscope yaw
    yaw_mag_lowpass = zeros(1, length(magnetometer_yaw));    % To store the low-pass filtered magnetometer yaw
    % Loop through each sample and apply the complementary filter
    for i = 2:length(magnetometer_yaw)
        dt = timestamp_vector(i) - timestamp_vector(i-1);  % Time step
        % High-pass filter for gyroscope yaw (integrated)
        yaw_gyro_highpass(i) = alpha_gyro * (yaw_combined(i-1) + gyroscope_yaw(i) * dt);
        % Low-pass filter for magnetometer yaw
        yaw_mag_lowpass(i) = alpha_mag * magnetometer_yaw(i);
        % Combine both with the complementary filter
        yaw_combined(i) = yaw_gyro_highpass(i) + yaw_mag_lowpass(i);
    end
    %Unwrap the yaw angles to ensure smooth transitions
    yaw_combined = unwrap(yaw_combined);
    yaw_gyro_highpass = unwrap(yaw_gyro_highpass);  % Unwrap gyroscope high-pass yaw
    yaw_mag_lowpass = unwrap(yaw_mag_lowpass);      % Unwrap magnetometer low-pass yaw
    figure;
    plot(timestamp_vector, yaw_mag_lowpass, 'r', 'DisplayName', 'Low-Pass Filter (Magnetometer)');
    hold on;
    plot(timestamp_vector, yaw_gyro_highpass, 'b', 'DisplayName', 'High-Pass Filter (Gyroscope)');
    plot(timestamp_vector, yaw_combined, 'g', 'DisplayName', 'Complementary Filter (Combined)');
    xlabel('Time (s)');
    ylabel('Yaw Angle (radians)');
    title('Low-Pass, High-Pass, and Complementary Filter Yaw');
    legend('show');
    grid on;
    hold off;
end

function plotIMUYawVsSensorFusionYaw(driving_imu_yaw, driving_yaw_after_comp_filter, driving_timestamp_vector, driving_num_samples)
    %Initialize array for storing IMU-computed yaw
    imu_yaw = zeros(1, driving_num_samples);
    % Loop through the data and compute yaw from quaternion orientation
    for i = 1:driving_num_samples
        % Extract quaternion components (x, y, z, w) from driving_imu_yaw
        qx = driving_imu_yaw(i).x;
        qy = driving_imu_yaw(i).y;
        qz = driving_imu_yaw(i).z;
        qw = driving_imu_yaw(i).w;
        % Ensure quaternion components are scalar
        if isscalar(qx) && isscalar(qy) && isscalar(qz) && isscalar(qw)
            % Convert quaternion to yaw using ZYX Euler angle representation
            imu_yaw(i) = atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy^2 + qz^2));
        else
            error('Quaternion components are not scalar values');
        end
    end
    %Unwrap the IMU yaw for smooth transitions (like we did for sensor fusion)
    imu_yaw = unwrap(imu_yaw);
    imu_yaw = wrapToPi(imu_yaw);
    figure;
    plot(driving_timestamp_vector(1:driving_num_samples), imu_yaw, 'r', 'LineWidth', 1.5, 'DisplayName', 'IMU Yaw (Original)');
    hold on;
    plot(driving_timestamp_vector(1:driving_num_samples), driving_yaw_after_comp_filter(1:driving_num_samples), 'g', 'LineWidth', 1.5, 'DisplayName', 'Sensor Fusion Yaw (Complementary Filter)');
    xlabel('Time (s)');
    ylabel('Yaw Angle (radians)');
    title('Comparison: IMU Yaw vs Sensor Fusion Yaw');
    legend('show');
    grid on;
    hold off;
end



%%% Main Script

% Load the data from the .mat file
data_going_in_circles = load('data_going_in_circles.mat');
data_driving = load('data_driving.mat');

% Define the number of samples based on the length of the imu_data field
circles_num_samples = length(data_going_in_circles.imu_data);
driving_num_samples = length(data_driving.imu_data);
driving_num_samples_gps = length(data_driving.gps_data);
sample_rate = 40;
% Conversion factor from tesla to milligauss
tesla_to_milligauss = 1e7;

% Initialize arrays for 2D magnetometer data (x, y) in milligauss
circles_magX_milligauss = zeros(1, circles_num_samples);
circles_magY_milligauss = zeros(1, circles_num_samples);

driving_magX_miligauss = zeros(1, driving_num_samples);
driving_magY_miligauss = zeros(1, driving_num_samples);

driving_timestamps = zeros(1, driving_num_samples);

driving_angular_velocity_Z = zeros(1, driving_num_samples);
driving_imu_yaw = struct('x', cell(1, driving_num_samples), 'y', cell(1, driving_num_samples), 'z', cell(1, driving_num_samples), 'w', cell(1, driving_num_samples));
driving_lateral_acceleration = zeros(1, driving_num_samples);
driving_forward_acceleration = zeros(1, driving_num_samples);

%array for driving gps data 
driving_gps_latitude = zeros(1, driving_num_samples_gps);
driving_gps_longitude = zeros(1, driving_num_samples_gps);
driving_gps_timestamps = zeros(1, driving_num_samples_gps);
driving_gps_easting = zeros(1, driving_num_samples_gps);
driving_gps_northing = zeros(1, driving_num_samples_gps);

% Convert all magnetometer readings from tesla to milligauss
for i = 1:circles_num_samples
    circles_magX_milligauss(i) = data_going_in_circles.imu_data{i}.mag_field.x * tesla_to_milligauss;
    circles_magY_milligauss(i) = data_going_in_circles.imu_data{i}.mag_field.y * tesla_to_milligauss;
end

for i = 1:driving_num_samples
    driving_magX_miligauss(i) = data_driving.imu_data{i}.mag_field.x * tesla_to_milligauss;
    driving_magY_miligauss(i) = data_driving.imu_data{i}.mag_field.y * tesla_to_milligauss;
    driving_timestamps(i) = data_driving.imu_data{i}.header.timestamp;
    driving_angular_velocity_Z(i) = data_driving.imu_data{i}.imu.angular_velocity.z;
    driving_forward_acceleration(i) = data_driving.imu_data{i}.imu.linear_acceleration.x;
    driving_lateral_acceleration(i) = data_driving.imu_data{i}.imu.linear_acceleration.y;
end

for i = 1:driving_num_samples
    % Extract quaternion orientation (x, y, z, w) from the IMU data
    driving_imu_yaw(i).x = data_driving.imu_data{i}.imu.orientation.x;
    driving_imu_yaw(i).y = data_driving.imu_data{i}.imu.orientation.y;
    driving_imu_yaw(i).z = data_driving.imu_data{i}.imu.orientation.z;
    driving_imu_yaw(i).w = data_driving.imu_data{i}.imu.orientation.w;
end

driving_timestamp_vector = driving_timestamps - driving_timestamps(1);


for i = 1:driving_num_samples_gps
    %extracting gps data 
    driving_gps_latitude(i) = data_driving.gps_data{i}.gps.latitude;
    driving_gps_longitude(i) = data_driving.gps_data{i}.gps.longitude;
    driving_gps_timestamps(i) = data_driving.gps_data{i}.header.timestamp;
    driving_gps_easting(i) = data_driving.gps_data{i}.gps.utm_easting;
    driving_gps_northing(i) = data_driving.gps_data{i}.gps.utm_northing;
end
    driving_gps_time_vector = driving_gps_timestamps - driving_gps_timestamps(1);


%Heading Estimate
% Heading Estimation : The magnetometer X-Y plot before and after hard and soft iron calibration
[magX_calibrated, magY_calibrated, offsetX_hard_iron, offsetY_hard_iron, scalingFactor] = calc_circles_magnetometer_calib(circles_magX_milligauss, circles_magY_milligauss);

% calibrate the raw yaw from data driving with offsets calculated from data going in circles
[driving_yaw_calibrated,driving_magX_corrected,driving_magY_corrected] = process_driving_magnetometer_data(driving_magX_miligauss, driving_magY_miligauss, offsetX_hard_iron, offsetY_hard_iron, scalingFactor, sample_rate);
plotMagnetometerTimeSeries(driving_timestamp_vector,driving_magX_miligauss, driving_magY_miligauss, driving_magX_corrected, driving_magY_corrected);

%Integrating yaw from gyro
[driving_yaw_angle] = integrateYawRate(driving_angular_velocity_Z, driving_timestamp_vector);

%Heading Estimation : Magnetometer Yaw & Yaw Integrated from Gyro together 
plotYawAngles(driving_yaw_calibrated, driving_yaw_angle, driving_timestamp_vector);

alpha_driving_gyro = 0.96;  % High-pass filter for gyroscope
alpha_driving_mag = 0.05;   % Low-pass filter for magnetometer

%Heading Estimation: LPF, HPF, and CF plots 
driving_yaw_after_comp_filter = complementaryFilterWithPlot(driving_yaw_calibrated, driving_yaw_angle, driving_timestamp_vector, alpha_driving_gyro, alpha_driving_mag);

%Heading Estimation: Yaw from the Complementary filter & Yaw angle computed by the IMU 
plotIMUYawVsSensorFusionYaw(driving_imu_yaw, driving_yaw_after_comp_filter, driving_timestamp_vector, driving_num_samples);

%Estimate Forward Velocity
driving_forward_velocity = cumtrapz(driving_timestamp_vector, driving_forward_acceleration);
% Plot the estimated velocity
figure;
plot(driving_timestamp_vector, driving_forward_velocity, 'b', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Estimated Forward Velocity from IMU Data');
grid on;

%Estimate Velocity from GPS
earth_radius = 6371000;
driving_gps_velocity = zeros(1, length(driving_gps_timestamps) - 1);

% Compute velocity from GPS data
for i = 2:length(driving_gps_timestamps)
    % Calculate differences in latitude and longitude in radians
    delta_lat = deg2rad(driving_gps_latitude(i) - driving_gps_latitude(i-1));
    delta_lon = deg2rad(driving_gps_longitude(i) - driving_gps_longitude(i-1));
    
    % Convert latitude to radians
    lat1 = deg2rad(driving_gps_latitude(i-1));
    lat2 = deg2rad(driving_gps_latitude(i));
    
    % Haversine formula
    a = sin(delta_lat/2)^2 + cos(lat1) * cos(lat2) * sin(delta_lon/2)^2;
    c = 2 * atan2(sqrt(a), sqrt(1-a));
    
    % Distance between two points in meters
    distance = earth_radius * c;
    
    % Time difference in seconds
    delta_time = driving_gps_time_vector(i) - driving_gps_time_vector(i-1);
    
    % Velocity in meters per second
    driving_gps_velocity(i-1) = distance / delta_time;
end

% Plot the GPS velocity
figure;
plot(driving_gps_time_vector(1:end-1), driving_gps_velocity, 'r', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Estimated Forward Velocity from GPS Data');
grid on;


%Interpolate GPS velocity to match IMU timestamps
gps_velocity_interp = interp1(driving_gps_time_vector(1:end-1), driving_gps_velocity, driving_timestamp_vector, 'linear', 'extrap');

%Plot IMU velocity and GPS velocity together
figure;
plot(driving_timestamp_vector, driving_forward_velocity, 'b', 'LineWidth', 1.5, 'DisplayName', 'IMU Integrated Velocity');
hold on;
plot(driving_timestamp_vector, gps_velocity_interp, 'r', 'LineWidth', 1.5, 'DisplayName', 'GPS Velocity');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Comparison of GPS and IMU Velocity Before Adjustment');
legend('show');
grid on;

%Estimate acceleration bias from IMU data
% Identify time points where velocity is near zero (from GPS data)
bias_threshold = 0.01; % Threshold for near-zero velocity in m/s
near_zero_velocity_indices = find(abs(gps_velocity_interp) < bias_threshold);

% Calculate the mean forward acceleration at near-zero velocity points
bias_acceleration = mean(driving_forward_acceleration(near_zero_velocity_indices));

% Correct forward acceleration by removing the bias
driving_forward_acceleration_corrected = driving_forward_acceleration - bias_acceleration;

driving_forward_acceleration_corrected(abs(driving_forward_acceleration_corrected) <= 0.33) = 0;

% Re-integrate corrected forward acceleration to get velocity
driving_forward_velocity_corrected = cumtrapz(driving_timestamp_vector, driving_forward_acceleration_corrected);

% Find a constant offset to match IMU data with GPS data
velocity_offset = mean(gps_velocity_interp) - mean(driving_forward_velocity_corrected);

% Apply the offset to the IMU velocity
driving_forward_velocity_corrected_shifted = driving_forward_velocity_corrected + velocity_offset;

% Plot the adjusted IMU velocity vs GPS velocity
figure;
plot(driving_timestamp_vector, driving_forward_velocity_corrected_shifted, 'b', 'LineWidth', 1.5, 'DisplayName', 'IMU Velocity (Shifted)');
hold on;
plot(driving_timestamp_vector, gps_velocity_interp, 'r', 'LineWidth', 1.5, 'DisplayName', 'GPS Velocity');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Comparison of GPS and IMU Velocity After Adjustment');
legend('show');
grid on;

% Dead Reckoning with IMU
driving_gps_displacement = zeros(1, driving_num_samples_gps);
for i = 2:driving_num_samples_gps
    lat1 = deg2rad(driving_gps_latitude(i-1));
    lon1 = deg2rad(driving_gps_longitude(i-1));
    lat2 = deg2rad(driving_gps_latitude(i));
    lon2 = deg2rad(driving_gps_longitude(i));

    %Haversine formula
    dlat = lat2 - lat1;
    dlon = lon2 - lon2;
    
    a = sin(dlat/2)^2 + cos(lat1) * cos(lat2) * sin(dlon/2)^2;
    c = 2 * atan2(sqrt(a), sqrt(1 - a));
    
    distance = earth_radius * c;
    
    % Accumulate displacement
    driving_gps_displacement(i) = driving_gps_displacement(i-1) + distance;
end

% Plot GPS displacement (Latitude and Longitude-based)
figure;
plot(driving_gps_time_vector, driving_gps_displacement, 'DisplayName', 'GPS Displacement (Lat/Lon)');
xlabel('Time (s)');
ylabel('Displacement (m)');
legend;
title('GPS Displacement (Latitude and Longitude-based)');
grid on;

driving_imu_displacement = cumtrapz(driving_timestamps, driving_forward_velocity_corrected_shifted);
figure;
plot(driving_timestamp_vector,driving_imu_displacement,'DisplayName','IMU displacement');
xlabel('Time (s)');
ylabel('Displacement (m)');
legend;
title('Displacement Obtained by Integrating Forward Velocity');
grid on;

% Plot both displacements on the same figure
figure;
plot(driving_gps_time_vector, driving_gps_displacement, 'DisplayName', 'GPS Displacement (Lat/Lon)', 'LineWidth', 1.5);
hold on;
plot(driving_timestamp_vector, driving_imu_displacement, 'DisplayName', 'Integrated IMU Displacement', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Displacement (m)');
legend;
title('Comparison of GPS and IMU Displacement');
grid on;


%Dead Reckoning With IMU
X = cumtrapz(driving_timestamp_vector, driving_forward_acceleration_corrected);
omega_X = driving_angular_velocity_Z .* X;
figure;
plot(driving_timestamp_vector, driving_lateral_acceleration, 'r', 'LineWidth', 1.5, 'DisplayName', 'y_{obs}');
hold on;
plot(driving_timestamp_vector, omega_X, 'b', 'LineWidth', 1.5, 'DisplayName', '\omega X');
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
title('Comparison of \omega X and y_{obs}');
legend('show');
grid on;

%Sample rate 
sample_rate = 1 / mean(diff(driving_timestamp_vector));  % Calculate the sample rate from the timestamp vector

%Cutoff frequency for the low-pass filter (in Hz)
cutoff_frequency = 0.7; 

%2nd-order Butterworth low-pass filter
[b, a] = butter(2, cutoff_frequency / (sample_rate / 2));  % Normalizing cutoff frequency

% Applying the filter to the noisy lateral acceleration (y_obs)
y_obs_filtered = filtfilt(b, a, driving_lateral_acceleration);

%Compute omega * X.
omega_X = driving_angular_velocity_Z .* X;

%Plot the results to compare omega * X with filtered y_obs
figure;
plot(driving_timestamp_vector, omega_X, 'b', 'LineWidth', 1.5, 'DisplayName', '\omega X');
hold on;
plot(driving_timestamp_vector, y_obs_filtered, 'r', 'LineWidth', 1.5, 'DisplayName', 'Filtered y_{obs}');
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
title('Comparison of \omega X and Filtered y_{obs}');
legend('show');
grid on;

% Time setup
imu_rate = 40; % IMU data rate in Hz
dt = 1 / imu_rate; % Time step
yaw = driving_yaw_calibrated;
velocity = driving_forward_velocity_corrected_shifted;

%Calculate initial GPS heading based on the first two points
initial_gps_yaw = atan2(driving_gps_northing(2) - driving_gps_northing(1), ...
                        driving_gps_easting(2) - driving_gps_easting(1));

%Apply initial yaw offset to align IMU and GPS trajectories
initial_imu_yaw = yaw(1);
yaw_offset = initial_gps_yaw - initial_imu_yaw;
aligned_yaw = yaw + yaw_offset;

%Identify the first turn in the GPS data
%Set a threshold for detecting a significant change in direction
turn_threshold = pi / 6; % 30 degrees in radians
gps_yaw = atan2(diff(driving_gps_northing), diff(driving_gps_easting));
first_turn_idx = find(abs(diff(gps_yaw)) > turn_threshold, 1, 'first');

%Calculate additional yaw correction at the first turn
if ~isempty(first_turn_idx)
    imu_turn_yaw = aligned_yaw(first_turn_idx + 1); % IMU yaw at the first turn
    gps_turn_yaw = gps_yaw(first_turn_idx);         % GPS yaw at the first turn
    additional_yaw_offset = gps_turn_yaw - imu_turn_yaw;
    
    % Apply additional yaw offset from the first turn onward
    aligned_yaw(first_turn_idx + 1:end) = aligned_yaw(first_turn_idx + 1:end) + additional_yaw_offset;
end

% Update velocity components with the corrected yaw
v_e = velocity .* cos(aligned_yaw); % Easting component
v_n = velocity .* sin(aligned_yaw); % Northing component

% Initialize position arrays and set the first IMU point to the GPS starting point
num_samples = length(v_e);
x_e = zeros(1, num_samples); % Easting position (IMU-based)
x_n = zeros(1, num_samples); % Northing position (IMU-based)
x_e(1) = driving_gps_easting(1);
x_n(1) = driving_gps_northing(1);

% Integrate (v_e, v_n) to estimate trajectory (x_e, x_n)
for i = 2:num_samples
    x_e(i) = x_e(i-1) + v_e(i) * dt;
    x_n(i) = x_n(i-1) + v_n(i) * dt;
end

% Set the first GPS point as the origin for relative plotting
gps_origin_easting = driving_gps_easting(1);
gps_origin_northing = driving_gps_northing(1);

% Convert GPS and IMU paths to relative coordinates
relative_gps_easting = driving_gps_easting - gps_origin_easting;
relative_gps_northing = driving_gps_northing - gps_origin_northing;
relative_x_e = x_e - gps_origin_easting;
relative_x_n = x_n - gps_origin_northing;

figure;
plot(relative_gps_easting, relative_gps_northing, 'b', 'LineWidth', 1.5); hold on;
plot(relative_x_e, relative_x_n, 'r', 'LineWidth', 1.5);
xlabel('Relative UTM Easting (m)');
ylabel('Relative UTM Northing (m)');
legend('GPS Trajectory', 'IMU Trajectory');
title('GPS Trajectory Vs IMU Trajectory');
grid on;
