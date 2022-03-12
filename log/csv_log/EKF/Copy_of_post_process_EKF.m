%% Post process MATLAB script for navigation code
clc,close all,clear all
%%read data
state_data = csvread('state_data.csv');
kalman_state_data = csvread('kalman_state_data.csv');

kalman_covariance_diagonal_data = csvread('kalman_covariance_diagonal.csv');


%% plot trajectory
title("Simulation trajectory visualisation")

plot3(state_data(:,2),state_data(:,3),state_data(:,4))
hold on
plot3(kalman_state_data(:,2),kalman_state_data(:,3),kalman_state_data(:,4))
legend("Simulated trajectory","Kalman Trajectory")

%% error plots
% time vectir
tt_kal = kalman_state_data(:,1);

% interpolate messages in a common time vector
interp_state_x = interp1(state_data(:,1),state_data(:,2),tt_kal);
interp_state_y = interp1(state_data(:,1),state_data(:,3),tt_kal);
interp_state_z = interp1(state_data(:,1),state_data(:,4),tt_kal);

interp_state_vel_x = interp1(state_data(:,1),state_data(:,5),tt_kal);
interp_state_vel_y = interp1(state_data(:,1),state_data(:,6),tt_kal);
interp_state_vel_z = interp1(state_data(:,1),state_data(:,7),tt_kal);


% calculate errors
err_x = kalman_state_data(:,2)-interp_state_x;
err_y = kalman_state_data(:,3)-interp_state_y;
err_z = kalman_state_data(:,4)-interp_state_z;

err_vel_x = kalman_state_data(:,5)-interp_state_vel_x;
err_vel_y = kalman_state_data(:,6)-interp_state_vel_y;
err_vel_z = kalman_state_data(:,7)-interp_state_vel_z;


% plot data
figure
tiledlayout(3,3)

nexttile
hold on
plot(tt_kal,err_x);
plot(tt_kal,err_y);
plot(tt_kal,err_z);
legend("Error in x axis","error in y axis","error in z axis")
title("Kalman error in position")
xlabel("time [s]")
ylabel("error [m]")

nexttile
hold on
plot(tt_kal,err_vel_x);
plot(tt_kal,err_vel_y);
plot(tt_kal,err_vel_z);
legend("Error in x axis","error in y axis","error in z axis")
title("Kalman error in velocity (Inertial frame)")
xlabel("time [s]")
ylabel("error [m/s]")

nexttile
hold on
plot(tt_kal,1.96*sqrt(kalman_covariance_diagonal_data(:,2)));
plot(tt_kal,1.96*sqrt(kalman_covariance_diagonal_data(:,3)));
plot(tt_kal,1.96*sqrt(kalman_covariance_diagonal_data(:,4)));
legend("error bounds in x axis","error bounds in y axis","error bounds in z axis")
title("Error bounds 95% confidance ")
xlabel("time [s]")
ylabel(" \pm error bounds 95% confidance [m]")



