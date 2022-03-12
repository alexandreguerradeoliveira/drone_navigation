%% Post process MATLAB script for navigation code
clc,close all,clear all
%%read data
state_data = csvread('state_data.csv');
kalman_state_data = csvread('kalman_state_data.csv');

%% trajectory plot
plot3(state_data(:,2),state_data(:,3),state_data(:,4))
hold on
plot3(kalman_state_data(:,2),kalman_state_data(:,3),kalman_state_data(:,4))

%% position error plots
% x chord error
figure

interp_state_x = interp1(state_data(:,1),state_data(:,2),kalman_state_data(:,1));
plot(kalman_state_data(:,1),kalman_state_data(:,2)-interp_state_x)
title("error in position in x cordinate")
xlabel("time [s]")
ylabel("error [m]")

% x chord error
figure

interp_state_y = interp1(state_data(:,1),state_data(:,3),kalman_state_data(:,1));
plot(kalman_state_data(:,1),kalman_state_data(:,3)-interp_state_y)
title("error in position in y cordinate")
xlabel("time [s]")
ylabel("error [m]")

% z chord error
figure

interp_state_z = interp1(state_data(:,1),state_data(:,4),kalman_state_data(:,1));
plot(kalman_state_data(:,1),kalman_state_data(:,4)-interp_state_z)
title("error in position in z cordinate")
xlabel("time [s]")
ylabel("error [m]")

%% vel error plots
