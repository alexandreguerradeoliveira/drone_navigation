clc,close all,clear all

% bag = rosbag("log_terrace_1.bag");
bag = rosbag("log.bag");
% bag = rosbag("log_terrace_vertical_mov.bag");

mag = select(bag,'Topic','/mavros/imu/mag');
mag_msg = readMessages(mag,'DataFormat','struct');

for k =1:size(mag_msg,1)
    mag_data_uncalibrated(k,:) = [mag_msg{k}.MagneticField.X,mag_msg{k}.MagneticField.Y,mag_msg{k}.MagneticField.Z];
end

[A,b,expmfs] = magcal(mag_data_uncalibrated,'sym')

mag_data_calibrated = (mag_data_uncalibrated-b)*A;

figure
plot3(mag_data_calibrated(:,1),mag_data_calibrated(:,2),mag_data_calibrated(:,3))
hold on
plot3(mag_data_uncalibrated(:,1),mag_data_uncalibrated(:,2),mag_data_uncalibrated(:,3))
legend("uncalibrated data","calibrated data")
axis equal



