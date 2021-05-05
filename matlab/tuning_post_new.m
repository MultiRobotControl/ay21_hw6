% ME4823 
% LT S. Royster
% HW6
% Spring 2021
clear all
close all
clc

%Load the file
<<<<<<< HEAD
fname = '2021-05-05-15-54-43.bag';       % Filename
=======
fname = '2021-05-04-20-42-13.bag';       % Filename
>>>>>>> 8c602bb3de0ecc093bbdb8e2823202591f8e9d42
%Create a bag file object with the file name
bag = rosbag(fname)
%Display a list of the topics and message types in the bag file
bag.AvailableTopics

%Create time series for the Odometry & Command data
%Retrieve the messages as a cell array
<<<<<<< HEAD
debug_vel_msgs = select(bag,'Topic','/cora/control/vel_pid_debug');
debug_yaw_msgs = select(bag,'Topic','/cora/control/yaw_pid_debug');
cmd_msgs = select(bag,'Topic','/cora/cmd_vel');
odom_msgs = select(bag,'Topic','/cora/sensors/p3d');
=======
odom_vel_msgs = select(bag,'Topic','/cora/control/vel_pid_debug');
odom_yaw_msgs = select(bag,'Topic','/cora/control/yaw_pid_debug');
cmd_msgs = select(bag,'Topic','/cora/cmd_vel');
>>>>>>> 8c602bb3de0ecc093bbdb8e2823202591f8e9d42
%Create a timeseries object of the subset of message fields we are interested in
odom_ts = timeseries(odom_msgs,'Pose.Pose.Position.X','Pose.Pose.Position.Y', ...
    'Pose.Pose.Orientation.W','Pose.Pose.Orientation.X','Pose.Pose.Orientation.Y','Pose.Pose.Orientation.Z', ...
    'Twist.Twist.Linear.X','Twist.Twist.Angular.Z');
cmd_ts = timeseries(cmd_msgs,'Linear.X','Linear.Y','Linear.Z','Angular.X','Angular.Y','Angular.Z');

%Plot Surge/Yaw Velocity vs. Setpoint/Goal
figure(1); clf();
%Plot the Data index corresponding to Twist.Twist.Linear.X
plot(odom_ts.Time-odom_ts.Time(1),odom_ts.Data(:,7))
hold on
%Plot the Data index corresponding to Linear.X
plot(cmd_ts.Time-odom_ts.Time(1), cmd_ts.Data(:,1),'ro')
xlabel('Time [s]')
ylabel('Surge Velocity [m/s]')
legend('Odom Velocity','Setpoint')
title('Achieved Surge Velocity vs. Commanded Velocity')
grid on

figure(2); clf();
%Plot the Data index corresponding to Twist.Twist.Angular.Z
plot(odom_ts.Time-odom_ts.Time(1),odom_ts.Data(:,8))
hold on

plot(cmd_ts.Time-odom_ts.Time(1), cmd_ts.Data(:,6),'ro')
xlabel('Time [s]')
ylabel('Yaw Velocity [rad/s]')
legend('Odom Velocity','Setpoint')
title('Achieved Yaw Velocity vs. Commanded Yaw Velocity')
grid on

%Plot the X,Y locations of the USV
figure(3); clf();
%Plot the Data index corresponding to Pose.Pose.Position.X and Pose.Pose.Position.Y
plot(odom_ts.Data(:,1),odom_ts.Data(:,2))
hold on
xlabel('X [m]')
ylabel('Y [m]')
text(odom_ts.Data(1,1),odom_ts.Data(1,2),'Start')
text(odom_ts.Data(end,1),odom_ts.Data(end,2),'Stop')
legend('USV Position','Location',"best")
title('USV XY Postion History')
grid on

%Plot USV Yaw Displacment vs. Time
%Convert Orientation from quaternion to Euler angles
% Note the convention for the quat2eul function is quaternion in order of WXYZ
q = odom_ts.Data(:,3:6);
e = quat2eul(q);
yaw = e(:,1);
yaw_d = rad2deg(yaw);
<<<<<<< HEAD

%Plot Yaw vs. Time
=======
Plot Yaw vs. Time
>>>>>>> 8c602bb3de0ecc093bbdb8e2823202591f8e9d42

figure(4); clf();
plot(odom_ts.Time-odom_ts.Time(1),yaw_d)
hold on
xlabel('Time [s]')
ylabel('Yaw Displacement [deg]')
legend('Yaw Displacement')
title('USV Yaw Displacment vs. Time')
grid on

%Qiver Plot
%Generate a quiver plot to illustrate the x/y position, heading and velocity - all on the same 2D graph. 
% Quiver Plot
x = odom_ts.Data(:,1); % x is the x position of the USV
y = odom_ts.Data(:,2); % y is the y position of the USV
vel = odom_ts.Data(:,7); % vel is the Linear.X velocity
yaw = yaw;                         % yaw is the heading angle of the USV in radians
 
u = vel.*cos(yaw);
v = vel.*sin(yaw);
 
figure(5); clf();
ii = 1:20:length(x);  % Decimate the data so that it plot only every Nth point.
quiver(x(ii),y(ii),u(ii),v(ii))
grid('on')
xlabel('X [m]')
ylabel('Y [m]')
title('Quiver Plot')

%Control Error Plots
%Resample and synchronize the two timeseries (odometry and commands) 
[synch_odom_ts,synch_cmd_ts]=synchronize(odom_ts,cmd_ts,'Union','InterpMethod','zoh');
%Plot the synchronized command/velocity vs. time and control error vs. time for both surge and yaw
figure(6); clf();
subplot(2,1,1)
plot(synch_cmd_ts.Time-synch_cmd_ts.Time(1),synch_cmd_ts.Data(:,1),'r--',...
    synch_odom_ts.Time-synch_cmd_ts.Time(1),synch_odom_ts.Data(:,7),'b-')
legend('SurgeCmd','SurgeVel')
xlabel('Time [s]');ylabel('Surge [m/s]')
title('Surge Command, Velocity and Control Error vs. Time')
grid on
subplot(2,1,2)
plot(synch_cmd_ts.Time-synch_cmd_ts.Time(1),synch_cmd_ts.Data(:,1)-synch_odom_ts.Data(:,7),'b-')
legend('Control Error')
xlabel('Time [s]');ylabel('Surge Error [m/s]')
grid on
figure(7); clf();
subplot(2,1,1)
plot(synch_cmd_ts.Time-synch_cmd_ts.Time(1),synch_cmd_ts.Data(:,6),'r--',...
    synch_odom_ts.Time-synch_cmd_ts.Time(1),synch_odom_ts.Data(:,8),'b-')
legend('YawVelCmd','YawVel','Location',"best")
xlabel('Time [s]');ylabel('YawVel [rad/s]')
title('Yaw Velocity Command, Yaw Velocity and Control Error vs. Time')
grid on
subplot(2,1,2)
plot(synch_cmd_ts.Time-synch_cmd_ts.Time(1),synch_cmd_ts.Data(:,6)-synch_odom_ts.Data(:,8),'b-')
legend('Control Error','Location',"best")
xlabel('Time [s]');ylabel('YawVel Error [rad/s]')
grid on