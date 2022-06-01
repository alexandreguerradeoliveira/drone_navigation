clc,close all,clear all

% bag = rosbag("log_terrace_1.bag");
bag = rosbag("log12_nomovement.bag");


mag = select(bag,'Topic','/mavros/imu/mag');
mag_msg = readMessages(mag,'DataFormat','struct');


imu_raw = select(bag,'Topic','/mavros/imu/data_raw');
imu_raw_msg = readMessages(imu_raw,'DataFormat','struct');

baro = select(bag,'Topic','/mavros/imu/static_pressure');
baro_msg = readMessages(baro,'DataFormat','struct');


for k =1:size(imu_raw_msg,1)
    omega_b(k,:)  = [imu_raw_msg{k}.AngularVelocity.X,imu_raw_msg{k}.AngularVelocity.Y,imu_raw_msg{k}.AngularVelocity.Z];
    acc_b(k,:) = [imu_raw_msg{k}.LinearAcceleration.X,imu_raw_msg{k}.LinearAcceleration.Y,imu_raw_msg{k}.LinearAcceleration.Z];

end

for k =1:size(mag_msg,1)
    mag_b(k,:) = [mag_msg{k}.MagneticField.X,mag_msg{k}.MagneticField.Y,mag_msg{k}.MagneticField.Z];
end

for k =1:size(baro_msg,1)
    delta_p_baro(k) = baro_msg{k}.FluidPressure-baro_msg{1}.FluidPressure;
end
g = 9.81;
rho = 1.225;

z_baro = delta_p_baro/(rho*g);


R_mag = diag([var(mag_b(:,1)),var(mag_b(:,2)),var(mag_b(:,3))])

R_baro = var(z_baro)

R_gyro = diag([var(omega_b(:,1)),var(omega_b(:,2)),var(omega_b(:,3))])

R_acc = diag([var(acc_b(:,1)),var(acc_b(:,2)),var(acc_b(:,3))])

