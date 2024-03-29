%% Post process MATLAB script for navigation code
clc,close all,clear all
%% Settings
calculate_cov_eul = 0; % set equals to zero to not calculate confidance interval of euler angles (takes a while)

%% read data
bag = rosbag("log.bag");

EKF_state = select(bag,'Topic','/kalman_rocket_state');
EKF_msg = readMessages(EKF_state,'DataFormat','struct');

sim_state = select(bag,'Topic','/rocket_state');
sim_msg = readMessages(sim_state,'DataFormat','struct');

EKF_cov_state = select(bag,'Topic','/process_cov');
EKF_cov_msg = readMessages(EKF_cov_state,'DataFormat','struct');


%%

for k = 1:1:min(size(EKF_cov_msg,1),size(EKF_msg,1))
    kalman_state_data(k,2:7+4+3) = [EKF_msg{k}.Pose.Position.X,EKF_msg{k}.Pose.Position.Y,EKF_msg{k}.Pose.Position.Z,EKF_msg{k}.Twist.Linear.X,EKF_msg{k}.Twist.Linear.Y,EKF_msg{k}.Twist.Linear.Z,EKF_msg{k}.Pose.Orientation.X,EKF_msg{k}.Pose.Orientation.Y,EKF_msg{k}.Pose.Orientation.Z,EKF_msg{k}.Pose.Orientation.W,EKF_msg{1}.Twist.Angular.X,EKF_msg{1}.Twist.Angular.Y,EKF_msg{1}.Twist.Angular.Z];
    baro_bias(k) = EKF_msg{k}.BarometerBias;
end

for k = 1:size(sim_msg,1)
    state_data(k,2:7+4+3) = [sim_msg{k}.Pose.Position.X,sim_msg{k}.Pose.Position.Y,sim_msg{k}.Pose.Position.Z,sim_msg{k}.Twist.Linear.X,sim_msg{k}.Twist.Linear.Y,sim_msg{k}.Twist.Linear.Z,sim_msg{k}.Pose.Orientation.X,sim_msg{k}.Pose.Orientation.Y,sim_msg{k}.Pose.Orientation.Z,sim_msg{k}.Pose.Orientation.W,sim_msg{1}.Twist.Angular.X,sim_msg{1}.Twist.Angular.Y,sim_msg{1}.Twist.Angular.Z];
end

for k = 1:min(size(EKF_cov_msg,1),size(EKF_msg,1))
    kalman_covariance_diagonal_data(k,:)= EKF_cov_msg{k}.Covariance; 
end

state_data(:,1) = sim_state.MessageList.Time-EKF_state.MessageList.Time(1);
kalman_state_data(:,1) = EKF_state.MessageList.Time(1:min(size(EKF_cov_msg,1),size(EKF_msg,1))) - EKF_state.MessageList.Time(1);


%% error plots
% time vector
tt_kal = kalman_state_data(:,1);

tt_kal = tt_kal(1:min(size(EKF_cov_msg,1),size(EKF_msg,1)));

% interpolate messages in a common time vector
interp_state_x = interp1(state_data(:,1),state_data(:,2),tt_kal);
interp_state_y = interp1(state_data(:,1),state_data(:,3),tt_kal);
interp_state_z = interp1(state_data(:,1),state_data(:,4),tt_kal);

interp_state_vel_x = interp1(state_data(:,1),state_data(:,5),tt_kal);
interp_state_vel_y = interp1(state_data(:,1),state_data(:,6),tt_kal);
interp_state_vel_z = interp1(state_data(:,1),state_data(:,7),tt_kal);

% interp_state_mass = interp1(state_data(:,1),state_data(:,15),tt_kal);

interp_state_om_x = interp1(state_data(:,1),state_data(:,12),tt_kal);
interp_state_om_y = interp1(state_data(:,1),state_data(:,13),tt_kal);
interp_state_om_z = interp1(state_data(:,1),state_data(:,14),tt_kal);

% calculate errors
err_x = kalman_state_data(:,2)-interp_state_x;
err_y = kalman_state_data(:,3)-interp_state_y;
err_z = kalman_state_data(:,4)-interp_state_z;

err_vel_x = kalman_state_data(:,5)-interp_state_vel_x;
err_vel_y = kalman_state_data(:,6)-interp_state_vel_y;
err_vel_z = kalman_state_data(:,7)-interp_state_vel_z;

% err_mass = kalman_state_data(:,15)-interp_state_mass;

err_om_x = kalman_state_data(:,12)-interp_state_om_x;
err_om_y = kalman_state_data(:,13)-interp_state_om_y;
err_om_z = kalman_state_data(:,14)-interp_state_om_z;



%%  error Plot config
figure
tiledlayout(3,3)

%Z_confidance = 2.576; % 99% interval
Z_confidance = 2;

%% err_x
nexttile
hold on
plot(tt_kal,err_x);
% plot(tt_kal,Z_confidance*sqrt(kalman_covariance_diagonal_data(:,2)),'red');
% plot(tt_kal,-Z_confidance*sqrt(kalman_covariance_diagonal_data(:,2)),'red');
xlabel("time [s]")
ylabel("error [m]")
title("Kalman error in position x")

%% err_y
nexttile
hold on
plot(tt_kal,err_y);
title("Kalman error in position y")
% plot(tt_kal,Z_confidance*sqrt(kalman_covariance_diagonal_data(:,3)),'red');
% plot(tt_kal,-Z_confidance*sqrt(kalman_covariance_diagonal_data(:,3)),'red');
xlabel("time [s]")
ylabel("error [m]")


%% err_z
nexttile
hold on
title("Kalman error in position z")
plot(tt_kal,err_z);
% plot(tt_kal,Z_confidance*sqrt(kalman_covariance_diagonal_data(:,4)),'red');
% plot(tt_kal,-Z_confidance*sqrt(kalman_covariance_diagonal_data(:,4)),'red');
xlabel("time [s]")
ylabel("error [m]")

%% err_vel_x
nexttile
hold on
plot(tt_kal,err_vel_x);
title("Kalman error in velocity x (Inertial frame)")
% plot(tt_kal,Z_confidance*sqrt(kalman_covariance_diagonal_data(:,5)),'red');
% plot(tt_kal,-Z_confidance*sqrt(kalman_covariance_diagonal_data(:,5)),'red');
xlabel("time [s]")
ylabel("error [m/s]")

%% err_vel_y
nexttile
hold on
plot(tt_kal,err_vel_y);
title("Kalman error in velocity y (Inertial frame)")
% plot(tt_kal,Z_confidance*sqrt(kalman_covariance_diagonal_data(:,6)),'red');
% plot(tt_kal,-Z_confidance*sqrt(kalman_covariance_diagonal_data(:,6)),'red');
xlabel("time [s]")
ylabel("error [m/s]")

%% err_vel_z
nexttile
hold on
plot(tt_kal,err_vel_z);
% plot(tt_kal,Z_confidance*sqrt(kalman_covariance_diagonal_data(:,7)),'red');
% plot(tt_kal,-Z_confidance*sqrt(kalman_covariance_diagonal_data(:,7)),'red');
title("Kalman error in velocity z (Inertial frame)")
xlabel("time [s]")
ylabel("error [m/s]")

%% 
% nexttile
% hold on
% plot(tt_kal,err_om_x);
% title("Kalman error in angular velocity x (Inertial frame)")
% plot(tt_kal,Z_confidance*sqrt(kalman_covariance_diagonal_data(:,12)),'red');
% plot(tt_kal,-Z_confidance*sqrt(kalman_covariance_diagonal_data(:,12)),'red');
% xlabel("time [s]")
% ylabel("error [rad/s]")
% 
% nexttile
% hold on
% plot(tt_kal,err_om_y);
% title("Kalman error in angular velocity y (Inertial frame)")
% plot(tt_kal,Z_confidance*sqrt(kalman_covariance_diagonal_data(:,13)),'red');
% plot(tt_kal,-Z_confidance*sqrt(kalman_covariance_diagonal_data(:,13)),'red');
% xlabel("time [s]")
% ylabel("error [rad/s]")
% 
% nexttile
% hold on
% plot(tt_kal,err_om_z);
% plot(tt_kal,Z_confidance*sqrt(kalman_covariance_diagonal_data(:,14)),'red');
% plot(tt_kal,-Z_confidance*sqrt(kalman_covariance_diagonal_data(:,14)),'red');
% title("Kalman error in angular velocity z (Inertial frame)")
% xlabel("time [s]")
% ylabel("error [rad/s]")

%% attitude

%get quaternion data from state
quat_kal = [kalman_state_data(:,11),kalman_state_data(:,8),kalman_state_data(:,9),kalman_state_data(:,10)]; % [qw,qx,qy,qz]

interp_qw = interp1(state_data(:,1),state_data(:,11),tt_kal);
interp_qx = interp1(state_data(:,1),state_data(:,8),tt_kal);
interp_qy = interp1(state_data(:,1),state_data(:,9),tt_kal);
interp_qz = interp1(state_data(:,1),state_data(:,10),tt_kal);
quat_sim = [interp_qw,interp_qx,interp_qy,interp_qz];% [qw,qx,qy,qz]

% calculate euler angles covariances based on quaternion covariance

syms q0 q1 q2 q3
f = jacobian([atan(2*(q0*q3+q1*q2)/(1-2*(q2^2 + q3^2)));asin(2*(q0*q2-q3*q1));atan((2*(q0*q1+q2*q3))/(1-2*(q1^2+q2^2)))],[q0 q1 q2 q3]);

if calculate_cov_eul ==1
    P_quat = zeros(4,4,size(kalman_covariance_diagonal_data(:,1),1));
    P_eul = zeros(3,3,size(kalman_covariance_diagonal_data(:,1),1));
    std_eul1 = zeros(1,size(kalman_covariance_diagonal_data(:,1),1));
    std_eul2 = zeros(1,size(kalman_covariance_diagonal_data(:,1),1));
    std_eul3 = zeros(1,size(kalman_covariance_diagonal_data(:,1),1));

    
    for k = 1:size(kalman_covariance_diagonal_data(:,1),1)
        %P_quat(:,:,k) = [kalman_quat_covariance_data(k,17),kalman_quat_covariance_data(k,14),kalman_quat_covariance_data(k,15),kalman_quat_covariance_data(k,16);...
        %                 kalman_quat_covariance_data(k,5),kalman_quat_covariance_data(k,2),kalman_quat_covariance_data(k,3),kalman_quat_covariance_data(k,4);...
        %                 kalman_quat_covariance_data(k,9),kalman_quat_covariance_data(k,6),kalman_quat_covariance_data(k,7),kalman_quat_covariance_data(k,8);...
        %                 kalman_quat_covariance_data(k,13),kalman_quat_covariance_data(k,10),kalman_quat_covariance_data(k,11),kalman_quat_covariance_data(k,12)];
        P_quat(:,:,k) = diag([kalman_covariance_diagonal_data(k,11),kalman_covariance_diagonal_data(k,8),kalman_covariance_diagonal_data(k,9),kalman_covariance_diagonal_data(k,10)]);
        P_quat(:,:,k) = (P_quat(:,:,k) + P_quat(:,:,k)')/2;
        Jac = subs(f,[q0 q1 q2 q3],[kalman_state_data(k,11),kalman_state_data(k,8),kalman_state_data(k,9),kalman_state_data(k,10)]);
        P_eul(:,:,k) = double(Jac*P_quat(:,:,k)*(Jac'));
   
        std_eul1(k) = sqrt(abs(P_eul(1,1,k)));
        std_eul2(k) = sqrt(abs(P_eul(2,2,k)));
        std_eul3(k) = sqrt(abs(P_eul(3,3,k)));
    end 
end


%transform to euler angles (ZYX euler: psi,theta,phi)

eul_sim = quat2eul(quat_sim)*(180/pi);
eul_kal= quat2eul(quat_kal)*(180/pi);

err_eul = (eul_kal-eul_sim);

nexttile
hold on
plot(tt_kal,err_eul(:,1));
title("Kalman error in euler \psi")
if calculate_cov_eul == 1
    plot(tt_kal,Z_confidance*std_eul1*(180/pi),'red');
    plot(tt_kal,-Z_confidance*std_eul1*(180/pi),'red');
end

xlabel("time [s]")
ylabel("error [deg]")

nexttile
hold on
plot(tt_kal,err_eul(:,2));
title("Kalman error in euler \theta")
if calculate_cov_eul == 1
    plot(tt_kal,Z_confidance*std_eul2*(180/pi),'red');
    plot(tt_kal,-Z_confidance*std_eul2*(180/pi),'red');
end

xlabel("time [s]")
ylabel("error [deg]")

nexttile
hold on
plot(tt_kal,err_eul(:,3));
title("Kalman error in euler \phi")
xlabel("time [s]")
ylabel("error [deg]")
if calculate_cov_eul == 1
    plot(tt_kal,Z_confidance*std_eul3*(180/pi),'red');
    plot(tt_kal,-Z_confidance*std_eul3*(180/pi),'red');
end

%%
%% Plots 
figure
tiledlayout(3,3)

nexttile
hold on
plot(tt_kal,interp_state_x);
plot(tt_kal,kalman_state_data(:,2));

plot(tt_kal,interp_state_x+Z_confidance*sqrt(kalman_covariance_diagonal_data(:,2)),'red');
plot(tt_kal,interp_state_x-Z_confidance*sqrt(kalman_covariance_diagonal_data(:,2)),'red');
xlabel("time [s]")
ylabel("position [m]")
title("position x")
legend("simulated","EKF","95% confidance")

nexttile
hold on
plot(tt_kal,interp_state_y);
plot(tt_kal,kalman_state_data(:,3));

title("position y")
plot(tt_kal,interp_state_y+Z_confidance*sqrt(kalman_covariance_diagonal_data(:,3)),'red');
plot(tt_kal,interp_state_y-Z_confidance*sqrt(kalman_covariance_diagonal_data(:,3)),'red');
xlabel("time [s]")
ylabel("position [m]")
legend("simulated","EKF","95% confidance")


nexttile
hold on
title("position z")
plot(tt_kal,interp_state_z);
plot(tt_kal,kalman_state_data(:,4));

plot(tt_kal,interp_state_z+Z_confidance*sqrt(kalman_covariance_diagonal_data(:,4)),'red');
plot(tt_kal,interp_state_z+-Z_confidance*sqrt(kalman_covariance_diagonal_data(:,4)),'red');
xlabel("time [s]")
ylabel("position [m]")
legend("simulated","EKF","95% confidance")

nexttile
hold on
plot(tt_kal,interp_state_vel_x);
plot(tt_kal,kalman_state_data(:,5));
title("Velocity x (Inertial frame)")
plot(tt_kal,interp_state_vel_x+Z_confidance*sqrt(kalman_covariance_diagonal_data(:,5)),'red');
plot(tt_kal,interp_state_vel_x-Z_confidance*sqrt(kalman_covariance_diagonal_data(:,5)),'red');
xlabel("time [s]")
ylabel("velocity [m/s]")
legend("simulated","EKF","95% confidance")
nexttile
hold on
plot(tt_kal,interp_state_vel_y);
plot(tt_kal,kalman_state_data(:,6));
title("Velocity y (Inertial frame)")
plot(tt_kal,interp_state_vel_y+Z_confidance*sqrt(kalman_covariance_diagonal_data(:,6)),'red');
plot(tt_kal,interp_state_vel_y-Z_confidance*sqrt(kalman_covariance_diagonal_data(:,6)),'red');
xlabel("time [s]")
ylabel("velocity [m/s]")
legend("simulated","EKF","95% confidance")
nexttile
hold on
plot(tt_kal,interp_state_vel_z);
plot(tt_kal,kalman_state_data(:,7));
plot(tt_kal,interp_state_vel_z+Z_confidance*sqrt(kalman_covariance_diagonal_data(:,7)),'red');
plot(tt_kal,interp_state_vel_z-Z_confidance*sqrt(kalman_covariance_diagonal_data(:,7)),'red');
title("Velocity z (Inertial frame)")
xlabel("time [s]")
ylabel("velocity [m/s]")
legend("simulated","EKF","95% confidance")

% Euler angles plot

nexttile
title("Euler angle: \psi")
hold on
plot(tt_kal,eul_sim(:,1));
plot(tt_kal,eul_kal(:,1));
xlabel('time [s]')
ylabel('angle [deg]')

if calculate_cov_eul == 1
    plot(tt_kal,eul_sim(:,1)+Z_confidance*std_eul1*(180/pi),'red');
    plot(tt_kal,eul_sim(:,1)-Z_confidance*std_eul1*(180/pi),'red');
legend("\psi_{sim}","\psi_{kal}","95% confidance")

else
    legend("\psi_{sim}","\psi_{kal}")
end




nexttile
title("Euler angle: \theta")
hold on
plot(tt_kal,eul_sim(:,2));
plot(tt_kal,eul_kal(:,2));
xlabel('time [s]')
ylabel('angle [deg]')

if calculate_cov_eul == 1
    plot(tt_kal,eul_sim(:,2)+Z_confidance*std_eul2*(180/pi),'red');
    plot(tt_kal,eul_sim(:,2)-Z_confidance*std_eul2*(180/pi),'red');
legend("\theta_{sim}","\theta_{kal}","95% confidance")

else
    legend("\theta_{sim}","\theta_{kal}")
end




nexttile
title("Euler angle: \theta")
hold on
plot(tt_kal,eul_sim(:,3));
plot(tt_kal,eul_kal(:,3));
xlabel('time [s]')
ylabel('angle [deg]')


if calculate_cov_eul == 1
    plot(tt_kal,eul_sim(:,3)+Z_confidance*std_eul3*(180/pi),'red');
    plot(tt_kal,eul_sim(:,3)-Z_confidance*std_eul3*(180/pi),'red');
    legend("\phi_{sim}","\phi_{kal}","95% confidance")

else
    legend("\phi_{sim}","\phi_{kal}")
end




%%

figure

hold on
title("altitude (position in z direction)")
plot(tt_kal,interp_state_z);
plot(tt_kal,kalman_state_data(:,4));

%plot(tt_kal,interp_state_z+Z_confidance*sqrt(kalman_covariance_diagonal_data(:,4)),'red');
%plot(tt_kal,interp_state_z+-Z_confidance*sqrt(kalman_covariance_diagonal_data(:,4)),'red');
xlabel("time [s]")
ylabel("position [m]")
% legend("simulated","EKF","95% confidance")
legend("simulated","EKF estimate")


figure
hold on
title("error in altitude z")
plot(tt_kal,err_z);

plot(tt_kal,Z_confidance*sqrt(kalman_covariance_diagonal_data(:,4)),'red');
plot(tt_kal,-Z_confidance*sqrt(kalman_covariance_diagonal_data(:,4)),'red');
xlabel("time [s]")
ylabel("position [m]")
legend("simulated","EKF","95% confidance")

figure

title("barometer bias")
plot(tt_kal,baro_bias);
xlabel("time [s]")
ylabel("bias [m]")
yline(0.1)
legend("estimated barometer bias","real bias")
