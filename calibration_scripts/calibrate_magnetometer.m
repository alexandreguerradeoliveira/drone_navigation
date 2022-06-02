clc,close all,clear all

%% select calibration data
bag = rosbag("log_exemple_magnetometer_calibration_data.bag");
%%


mag = select(bag,'Topic','/mavros/imu/mag');
mag_msg = readMessages(mag,'DataFormat','struct');

for k =1:size(mag_msg,1)
    mag_data_uncalibrated(k,:) = [mag_msg{k}.MagneticField.X,mag_msg{k}.MagneticField.Y,mag_msg{k}.MagneticField.Z];
end

%% The actual calibration
[A,b,expmfs] = magcal(mag_data_uncalibrated,'sym')

%% Plot the results
mag_data_calibrated = (mag_data_uncalibrated-b)*A;

figure
plot3(mag_data_calibrated(:,1),mag_data_calibrated(:,2),mag_data_calibrated(:,3))
hold on
plot3(mag_data_uncalibrated(:,1),mag_data_uncalibrated(:,2),mag_data_uncalibrated(:,3))
legend("uncalibrated data","calibrated data")
xlabel("x axis [T]")
ylabel("y axis [T]")
zlabel("z axis [T]")
title("Magnetometer calibration data")
axis equal



