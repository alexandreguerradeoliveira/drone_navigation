clc,close all,clear all

bag = rosbag("log.bag");

EKF_state = select(bag,'Topic','/kalman_rocket_state');
EKF_msg = readMessages(EKF_state,'DataFormat','struct');

px4_state = select(bag,'Topic','/mavros/odometry/in');
px4_msg = readMessages(px4_state,'DataFormat','struct');

gps_state = select(bag,'Topic','/mavros/global_position/raw/fix');
gps_msg = readMessages(gps_state,'DataFormat','struct');

baro = select(bag,'Topic','/mavros/imu/static_pressure');
baro_msg = readMessages(baro,'DataFormat','struct');
for k =1:size(baro_msg,1)
    p_baro(k) = baro_msg{k}.FluidPressure;
    delta_p_baro(k) = -baro_msg{k}.FluidPressure+baro_msg{1}.FluidPressure;
end
g = 9.81;
rho = 1.225;

z_baro = delta_p_baro/(rho*g);
tt_baro = baro.MessageList.Time-EKF_state.MessageList.Time(1);

tt_EKF = EKF_state.MessageList.Time-EKF_state.MessageList.Time(1);
tt_gps = gps_state.MessageList.Time-EKF_state.MessageList.Time(1);
tt_px4 = px4_state.MessageList.Time-EKF_state.MessageList.Time(1);


[kx,ky] = lat_long_coeffs(gps_msg{1}.Latitude);

for k = 1:size(EKF_msg,1)
EKF_pos_x(k) = EKF_msg{k}.Pose.Position.X;
EKF_pos_y(k) = EKF_msg{k}.Pose.Position.Y;
EKF_pos_z(k) = EKF_msg{k}.Pose.Position.Z;

quat_EKF(:,k) = [EKF_msg{k}.Pose.Orientation.W;EKF_msg{k}.Pose.Orientation.X;EKF_msg{k}.Pose.Orientation.Y;EKF_msg{k}.Pose.Orientation.Z];

eul_kal(k,:)= quat2eul(quat_EKF(:,k)')*(180/pi);

end


for k = 1:size(px4_msg,1)
px4_pos_x(k) = px4_msg{k}.Pose.Pose.Position.Y-px4_msg{1}.Pose.Pose.Position.Y;
px4_pos_y(k) = (px4_msg{k}.Pose.Pose.Position.X-px4_msg{1}.Pose.Pose.Position.X);
px4_pos_z(k) = px4_msg{k}.Pose.Pose.Position.Z-px4_msg{1}.Pose.Pose.Position.Z;

quat_px4(:,k) = [px4_msg{k}.Pose.Pose.Orientation.W;px4_msg{k}.Pose.Pose.Orientation.X;px4_msg{k}.Pose.Pose.Orientation.Y;px4_msg{k}.Pose.Pose.Orientation.Z];

eul_px4(k,:)= quat2eul(quat_px4(:,k)')*(180/pi);
end

for k = 1:size(gps_msg,1)
    
    % position using linearisation
gps_pos_x(k) = kx*(gps_msg{k}.Latitude-gps_msg{1}.Latitude);
gps_pos_y(k) = ky*(gps_msg{k}.Longitude-gps_msg{1}.Longitude);
gps_pos_z(k) = gps_msg{k}.Altitude - gps_msg{1}.Altitude;

% position using matlab function

lla(k,:) = [gps_msg{k}.Latitude,gps_msg{k}.Longitude,gps_msg{k}.Altitude];
lla0 = [lla(1,1),lla(1,2)];

flatearth_pos(k,:) = lla2flat(lla(k,:), lla0, 0, 0);


end




%%

% eul_px4(:,1) = eul_px4(:,1) - ones(size(eul_px4(:,1)))*eul_px4(1,1);
 eul_px4(:,1) = eul_px4(:,1) - ones(size(eul_px4(:,1)))*(-eul_kal(1,1)+eul_px4(1,1));

figure
hold on
plot(tt_EKF,eul_kal(:,1))
plot(tt_EKF,eul_kal(:,2))
plot(tt_EKF,eul_kal(:,3))

plot(tt_px4,eul_px4(:,1),'--')
plot(tt_px4,eul_px4(:,2),'--')
plot(tt_px4,eul_px4(:,3),'--')
legend("\psi_{ROS}","\theta_{ROS}","\phi_{ROS}","\psi_{px4}","\theta_{px4}","\phi_{px4}")


% legend("\psi_{ROS}","\theta_{ROS}","\phi_{ROS}")
xlabel("time [s]")
ylabel("angle [deg]")
title("Euler angles as a function of time")

%%

title("3d trajectory")

figure
plot3(EKF_pos_x,EKF_pos_y,EKF_pos_z)
hold on
plot3(px4_pos_x,px4_pos_y,px4_pos_z)
plot3(gps_pos_x,gps_pos_y,gps_pos_z)

legend("ros EKF trajectory","px4 EKF trajectory","gps trajectory");
axis equal
xlabel("x axis position [m]")
ylabel("y axis position [m]")
zlabel("z axis position [m]")



%%
figure
hold on
plot(tt_baro,z_baro);
plot(tt_gps,gps_pos_z);
plot(tt_EKF,EKF_pos_z);
plot(tt_px4,px4_pos_z);




xlabel("time [s]")
ylabel("altitude[m]")
legend("raw barometer altitude","raw gps altitude","Ros EKF ","px4 EKF")

%%
figure
title("ground path")
plot(EKF_pos_x,EKF_pos_y)
hold on
plot(px4_pos_x,px4_pos_y)
plot(gps_pos_x,gps_pos_y)
legend("ros EKF trajectory","px4 EKF trajectory","gps trajectory");
axis equal
xlabel("x axis position [m]")
ylabel("y axis position [m]")


% gps linearisation
function [kx,ky] = lat_long_coeffs(lat)
    lat = lat*pi/180;
    kx = 111132.92-559.82*cos(2*lat) + 1.175*cos(4*lat)-0.0023*cos(6*lat);
    ky = (111412.84*cos(lat)-93.5*cos(3*lat)+0.118*cos(5*lat));

end

