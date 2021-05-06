%% ME4823 Assignment 6
%% Load .bag file and create bag file object. 

fname = '2021-05-05-21-21-29.bag'; 
bag = rosbag(fname);
 
% Display available topics and message types in bag file. 
bag.AvailableTopics

% Create a time series of the Odometry data
% Retrieve the messages as a cell array
odom_msgs = select(bag,'Topic','/cora/sensors/p3d');
 
% Create a timeseries object of the subset of message fields we are interested in
odom_ts = timeseries(odom_msgs,'Pose.Pose.Position.X','Pose.Pose.Position.Y', ...
    'Pose.Pose.Orientation.W','Pose.Pose.Orientation.X','Pose.Pose.Orientation.Y',...
    'Pose.Pose.Orientation.Z','Twist.Twist.Linear.X','Twist.Twist.Angular.Z');

% Create a time series of the Cmd data
% Retrieve the messages as a cell array
cmd_msgs = select(bag,'Topic','/cora/cmd_vel');
 
cmd_ts = timeseries(cmd_msgs,'Linear.X','Linear.Y','Linear.Z',...
        'Angular.X','Angular.Y','Angular.Z');

%% 1) Plot Surge Velocity and Setpoint/Goal

figure(1); clf();
% Plot the Data index corresponding to Twist.Twist.Linear.X
plot(odom_ts.Time,odom_ts.Data(:,7))
hold on
% Plot the Data index corresponding to Linear.X
plot(cmd_ts.Time, cmd_ts.Data(:,1),'ro')
title('(1) Estimated Surge Velocity and Surge Commands')
xlabel('Time [s]')
ylabel('Surge Velocity [m/s]')
legend('Odom Velocity','Setpoint')
grid on
%% 2) Plot Yaw Velocity and Setpoint/Goal

figure(2); clf();
% Plot the Data index corresponding to Twist.Twist.Angular.Z
plot(odom_ts.Time,odom_ts.Data(:,8))
hold on
% Plot the Data index corresponding to Linear.X
plot(cmd_ts.Time, cmd_ts.Data(:,6),'ro')
title('(2) Estimated Yaw Velocity and Yaw Commands')
xlabel('Time [s]')
ylabel('Yaw Velocity [rad/s]')
legend('Yaw Velocity','Setpoint')
grid on
%% 3) Plot X-Y Position of USV

figure(3); clf();
% Plot the Data index corresponding to 'Pose.Pose.Position.X' & 'Pose.Pose.Position.Y'
plot(odom_ts.Data(:,1),odom_ts.Data(:,2))
title('(3) Position of USV')
xlabel('X')
ylabel('Y')
grid on
%% 4) Quiver plot

% Convert quaternion to Euler angle
% Note the convention for the quat2eul function is quaternion in order of WXYZ
q = odom_ts.Data(:,3:6);
e = quat2eul(q);
yaw = e(:,1);

% Quiver Plot 
vel = odom_ts.Data(:,7);
x = odom_ts.Data(:,1); 
y = odom_ts.Data(:,2); 
u = vel.*cos(yaw);
v = vel.*sin(yaw);
 
figure(4); clf();
ii = 1:20:length(x);  % Decimate the data so that it plot only every 20th point.
quiver(x(ii),y(ii),u(ii),v(ii))
grid('on')
title('(5) Quiver Plot of USV')
xlabel('X [m]')
ylabel('Y [m]')
%% 5) Interpolate and Trim plots of surge velocities

% Plot surge against time.
common_cmd = interp1(cmd_ts.Time, cmd_ts.Data(:,1), odom_ts.Time,"previous");
ind = ~isnan(common_cmd); % Find NaN, assign 0, otherwise 1 to omit NaN.

figure(5)
subplot(2,1,1)
plot(odom_ts.Time(ind), odom_ts.Data(ind,7), odom_ts.Time(ind), common_cmd(ind), 'r--'); 
grid('on')
title('(5.a) Interpolate and Trim of Surge')
ylabel('Surge [m/s]')
xlabel('Time [s]')
legend('Odom Velocity','Setpoint')

err = common_cmd(ind) - odom_ts.Data(ind,7); 

subplot(2,1,2)
plot(odom_ts.Time(ind), err)
grid('on')
title('(5.b) Interpolate and Trim of Surge Error')
ylabel('Surge Error [m/s]')
xlabel('Time [s]')
%% 6) Interpolate and Trim plots of yaw velocities

% Plot yaw against time.
common_yaw = interp1(cmd_ts.Time, cmd_ts.Data(:,6), odom_ts.Time,"previous");
ind2 = ~isnan(common_yaw); % Find NaN, assign 0, otherwise 1 to omit NaN.

figure(6)
subplot(2,1,1)
plot(odom_ts.Time(ind2), odom_ts.Data(ind2,8), odom_ts.Time(ind2), common_yaw(ind2), 'r--'); 
grid('on')
title('(6.a) Interpolate and Trim of Yaw Velocity')
ylabel('Yaw Velocity [rad/s]')
xlabel('Time [s]')
legend('Odom Velocity','Setpoint','Location',"best")

err2 = common_yaw(ind2) - odom_ts.Data(ind2,8); 

subplot(2,1,2)
plot(odom_ts.Time(ind2), err2)
grid('on')
title('(6.b) Interpolate and Trim of Yaw Error')
ylabel('Yaw Error [rad/s]')
xlabel('Time [s]')
