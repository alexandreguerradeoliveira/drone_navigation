%% Post process MATLAB script for navigation code
% clc,close all,clear all
% %%read data
% state_data = csvread('state_data.csv');
% kalman_state_data = csvread('kalman_state_data.csv');
% 
% kalman_covariance_diagonal_data = csvread('kalman_covariance_diagonal.csv');
clc,close all,clear all
%% Settings
calculate_cov_eul = 0; % set equals to zero to not calculate confidance interval of euler angles (takes a while)

%% read data
% state_data = csvread('state_data.csv');
% kalman_state_data = csvread('kalman_state_data.csv');
% 
% kalman_covariance_diagonal_data = csvread('kalman_covariance_diagonal.csv');
% kalman_quat_covariance_data = csvread('kalman_quat_covariance_diagonal.csv');

bag = rosbag("log.bag");
% bag = rosbag("log_terrace_vertical_mov.bag");

EKF_state = select(bag,'Topic','/kalman_rocket_state');
EKF_msg = readMessages(EKF_state,'DataFormat','struct');

sim_state = select(bag,'Topic','/rocket_state');
sim_msg = readMessages(sim_state,'DataFormat','struct');

EKF_cov_state = select(bag,'Topic','/process_cov');
EKF_cov_msg = readMessages(EKF_cov_state,'DataFormat','struct');


state_data(:,1) = sim_state.MessageList.Time-EKF_state.MessageList.Time(1);
kalman_state_data(:,1) = EKF_state.MessageList.Time - EKF_state.MessageList.Time(1);
%%

for k = 1:size(EKF_msg,1)
    kalman_state_data(k,2:7+4+3) = [EKF_msg{k}.Pose.Position.X,EKF_msg{k}.Pose.Position.Y,EKF_msg{k}.Pose.Position.Z,EKF_msg{k}.Twist.Linear.X,EKF_msg{k}.Twist.Linear.Y,EKF_msg{k}.Twist.Linear.Z,EKF_msg{k}.Pose.Orientation.X,EKF_msg{k}.Pose.Orientation.Y,EKF_msg{k}.Pose.Orientation.Z,EKF_msg{k}.Pose.Orientation.W,EKF_msg{1}.Twist.Angular.X,EKF_msg{1}.Twist.Angular.Y,EKF_msg{1}.Twist.Angular.Z];
end

for k = 1:size(sim_msg,1)
    state_data(k,2:7+4+3) = [sim_msg{k}.Pose.Position.X,sim_msg{k}.Pose.Position.Y,sim_msg{k}.Pose.Position.Z,sim_msg{k}.Twist.Linear.X,sim_msg{k}.Twist.Linear.Y,sim_msg{k}.Twist.Linear.Z,sim_msg{k}.Pose.Orientation.X,sim_msg{k}.Pose.Orientation.Y,sim_msg{k}.Pose.Orientation.Z,sim_msg{k}.Pose.Orientation.W,sim_msg{1}.Twist.Angular.X,sim_msg{1}.Twist.Angular.Y,sim_msg{1}.Twist.Angular.Z];
end

for k = 1:size(EKF_cov_msg,1)
    kalman_covariance_diagonal_data(k,:)= EKF_cov_msg{k}.Covariance; 
end

%% plot trajectory
title("Simulation trajectory visualisation")
axis equal
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

% interp_state_mass = interp1(state_data(:,1),state_data(:,15),tt_kal);

% calculate errors
err_x = kalman_state_data(:,2)-interp_state_x;
err_y = kalman_state_data(:,3)-interp_state_y;
err_z = kalman_state_data(:,4)-interp_state_z;

err_vel_x = kalman_state_data(:,5)-interp_state_vel_x;
err_vel_y = kalman_state_data(:,6)-interp_state_vel_y;
err_vel_z = kalman_state_data(:,7)-interp_state_vel_z;

% err_mass = kalman_state_data(:,15)-interp_state_mass;


%%  error Plot config
figure
tiledlayout(4,3)


Z_confidance = 2.576; % 99% interval

%% err_x
nexttile
hold on
plot(tt_kal,err_x);
plot(tt_kal,Z_confidance*sqrt(kalman_covariance_diagonal_data(:,2)),'red');
plot(tt_kal,-Z_confidance*sqrt(kalman_covariance_diagonal_data(:,2)),'red');
xlabel("time [s]")
ylabel("error [m]")
title("Kalman error in position x")

%% err_y
nexttile
hold on
plot(tt_kal,err_y);
title("Kalman error in position y")
plot(tt_kal,Z_confidance*sqrt(kalman_covariance_diagonal_data(:,3)),'red');
plot(tt_kal,-Z_confidance*sqrt(kalman_covariance_diagonal_data(:,3)),'red');
xlabel("time [s]")
ylabel("error [m]")


%% err_z
nexttile
hold on
title("Kalman error in position z")
plot(tt_kal,err_z);
plot(tt_kal,Z_confidance*sqrt(kalman_covariance_diagonal_data(:,4)),'red');
plot(tt_kal,-Z_confidance*sqrt(kalman_covariance_diagonal_data(:,4)),'red');
xlabel("time [s]")
ylabel("error [m]")

%% err_vel_x
nexttile
hold on
plot(tt_kal,err_vel_x);
title("Kalman error in velocity x (Inertial frame)")
plot(tt_kal,Z_confidance*sqrt(kalman_covariance_diagonal_data(:,5)),'red');
plot(tt_kal,-Z_confidance*sqrt(kalman_covariance_diagonal_data(:,5)),'red');
xlabel("time [s]")
ylabel("error [m/s]")

%% err_vel_y
nexttile
hold on
plot(tt_kal,err_vel_y);
title("Kalman error in velocity y (Inertial frame)")
plot(tt_kal,Z_confidance*sqrt(kalman_covariance_diagonal_data(:,6)),'red');
plot(tt_kal,-Z_confidance*sqrt(kalman_covariance_diagonal_data(:,6)),'red');
xlabel("time [s]")
ylabel("error [m/s]")

%% err_vel_z
nexttile
hold on
plot(tt_kal,err_vel_z);
plot(tt_kal,Z_confidance*sqrt(kalman_covariance_diagonal_data(:,7)),'red');
plot(tt_kal,-Z_confidance*sqrt(kalman_covariance_diagonal_data(:,7)),'red');
title("Kalman error in velocity z (Inertial frame)")
xlabel("time [s]")
ylabel("error [m/s]")

%% attitude

%get quaternion data from state
quat_kal = [kalman_state_data(:,11),kalman_state_data(:,8),kalman_state_data(:,9),kalman_state_data(:,10)]; % [qw,qx,qy,qz]

interp_qw = interp1(state_data(:,1),state_data(:,11),tt_kal);
interp_qx = interp1(state_data(:,1),state_data(:,8),tt_kal);
interp_qy = interp1(state_data(:,1),state_data(:,9),tt_kal);
interp_qz = interp1(state_data(:,1),state_data(:,10),tt_kal);
quat_sim = [interp_qw,interp_qx,interp_qy,interp_qz];% [qw,qx,qy,qz]

%transform to euler angles

rod_sim = quat2rod(quat_sim);
rod_kal= quat2rod(quat_kal);

err_rod = (rod_kal-rod_sim);

nexttile
hold on
plot(tt_kal,err_rod(:,1));
title("error in alpha_x ")
xlabel("time [s]")
ylabel("error [deg]")
plot(tt_kal,Z_confidance*sqrt(kalman_covariance_diagonal_data(:,8)),'red');
plot(tt_kal,-Z_confidance*sqrt(kalman_covariance_diagonal_data(:,8)),'red');

nexttile
hold on
plot(tt_kal,err_rod(:,2));
title("error in alpha_y ")
xlabel("time [s]")
ylabel("error [deg]")

plot(tt_kal,Z_confidance*sqrt(kalman_covariance_diagonal_data(:,9)),'red');
plot(tt_kal,-Z_confidance*sqrt(kalman_covariance_diagonal_data(:,9)),'red');

nexttile
hold on
plot(tt_kal,err_rod(:,3));
title("error in alpha_z")
xlabel("time [s]")
ylabel("error [deg]")
plot(tt_kal(1:end-3),Z_confidance*sqrt(kalman_covariance_diagonal_data(1:end-3,10)),'red');
plot(tt_kal(1:end-3),-Z_confidance*sqrt(kalman_covariance_diagonal_data(1:end-3,10)),'red');


%% err_mass
% nexttile
% hold on
% plot(tt_kal,err_mass);
% plot(tt_kal,Z_confidance*sqrt(kalman_covariance_diagonal_data(:,14)),'red');
% plot(tt_kal,-Z_confidance*sqrt(kalman_covariance_diagonal_data(:,14)),'red');
% title("Kalman error in mass")
% xlabel("time [s]")
% ylabel("error [Kg]")

%% Plots
figure
title("Rodrigues parameters")
hold on
plot(tt_kal,rod_sim(:,1));
plot(tt_kal,rod_sim(:,2));
plot(tt_kal,rod_sim(:,3));
plot(tt_kal,rod_kal(:,1));
plot(tt_kal,rod_kal(:,2));
plot(tt_kal,rod_kal(:,3));
legend("\alpha_{x,sim}","\alpha_{y,sim}","\alpha_{z,sim}","\alpha_{x,kal}","\alpha_{y,kal}","\alpha_{z,kal}")







