clc,close all,clear all

%% Chose the data to be read here
bag = rosbag("log_exemple_covariance_calculation.bag");

%% extract data from the sensors
mag = select(bag,'Topic','/mavros/imu/mag');
mag_msg = readMessages(mag,'DataFormat','struct');


imu_raw = select(bag,'Topic','/mavros/imu/data_raw');
imu_raw_msg = readMessages(imu_raw,'DataFormat','struct');

baro = select(bag,'Topic','/mavros/imu/static_pressure');
baro_msg = readMessages(baro,'DataFormat','struct');


% gyroscope and accelerometer data
for k =1:size(imu_raw_msg,1)
    omega_b(k,:)  = [imu_raw_msg{k}.AngularVelocity.X,imu_raw_msg{k}.AngularVelocity.Y,imu_raw_msg{k}.AngularVelocity.Z];
    acc_b(k,:) = [imu_raw_msg{k}.LinearAcceleration.X,imu_raw_msg{k}.LinearAcceleration.Y,imu_raw_msg{k}.LinearAcceleration.Z];

end

% magnetometer data
for k =1:size(mag_msg,1)
    mag_b(k,:) = [mag_msg{k}.MagneticField.X,mag_msg{k}.MagneticField.Y,mag_msg{k}.MagneticField.Z];
    mag_b(k,:) = mag_b(k,:)/norm(mag_b(k,:)); % normalise the magnetic field (the normalised version is what is used in the EKF to find the direction of north)
end

% barometer data
for k =1:size(baro_msg,1)
    delta_p_baro(k) = baro_msg{k}.FluidPressure-baro_msg{1}.FluidPressure;
end
g = 9.81;
rho = 1.225;
z_baro = delta_p_baro/(rho*g); % trasnform pressure readings of the barometer into altititude mesurements using the same model as the EKF

%% Calculate the covariance matrices
R_mag = diag([var(mag_b(:,1)),var(mag_b(:,2)),var(mag_b(:,3))])

R_baro = var(z_baro)

R_gyro = diag([var(omega_b(:,1)),var(omega_b(:,2)),var(omega_b(:,3))])

R_acc = diag([var(acc_b(:,1)),var(acc_b(:,2)),var(acc_b(:,3))])

