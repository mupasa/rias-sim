%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Read .bag files
% Sangjun Lee
% 8/30/2018
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear; close all; clc;

%%  Copy bag file from Jackal to lacal folder
% scp administrator@192.168.1.104:/home/administrator/bagfiles/2018-08-30-16-34-13.bag /Users/SangjunLee/Mupasa/BoilerMaker/Research/mySimulation/playjackal/bagfiles

%%  Select bag file
[fileName,pathName] = uigetfile({'data/*.bag'},'Select bag file');
filePath = strcat(pathName,fileName);

%%  Open rosbag 
bag     = rosbag(filePath);                     % open bag file
t0      = bag.StartTime;                        % get initial time
bag.AvailableTopics                             % display all recorded topics

%%  Extract messages
% get individual topic bags
bag_imu      = select(bag,'Topic','/imu/data');
bag_odom     = select(bag,'Topic','/jackal_velocity_controller/odom');
bag_cmdvel   = select(bag,'Topic','/jackal_velocity_controller/cmd_vel');
bag_img      = select(bag,'Topic','/axis/image_raw/compressed');

% all message data
imu             = readMessages(bag_imu);
odom            = readMessages(bag_odom);
cmdvel          = readMessages(bag_cmdvel);
compressed_imgs = readMessages(bag_img);

% number of messages
n_imu_msg        = size(imu,1);
n_odom_msg       = size(odom,1);
n_cmdvel_msg     = size(cmdvel,1);
n_img_msg        = size(compressed_imgs,1);

%%  Process imu data
if n_imu_msg > 0
    t_imu   = bag_imu.timeseries.Time - t0;     % time series
    f_imu   = 1/mean(diff(t_imu));              % average sampling frequency
    a_x     = zeros(1,n_imu_msg);
    a_y     = zeros(1,n_imu_msg);  
    yaw     = zeros(1,n_imu_msg);
    w_z     = zeros(1,n_imu_msg);
    for i = 1:n_imu_msg
        a_x(i)  = imu{i}.LinearAcceleration.X;
        a_y(i)  = imu{i}.LinearAcceleration.Y;
        w_z(i)  = imu{i}.AngularVelocity.Z;
        eul     = quat2eul([imu{i}.Orientation.X,...
                                imu{i}.Orientation.Y,...
                                imu{i}.Orientation.Z,...
                                imu{i}.Orientation.W]);
        yaw(i)  = eul(3);
    end
end

%%  Process odometry data
if n_odom_msg > 0
    t_odom   = bag_odom.timeseries.Time - t0;     % time series
    f_odom   = 1/mean(diff(t_odom));              % average sampling frequency
    pose_x     = zeros(1,n_odom_msg);
    pose_y     = zeros(1,n_odom_msg);  
    for i = 1:n_odom_msg
        pose_x(i)  = odom{i}.Pose.Pose.Position.X;
        pose_y(i)  = odom{i}.Pose.Pose.Position.Y;
    end
end

%%  Process image data
if n_img_msg > 0
    t_img           = bag_img.timeseries.Time - t0;     % time series
    f_img           = 1/mean(diff(t_img));              % average sampling frequency
    img0            = readImage(compressed_imgs{1});
    height          = size(img0,1);
    width           = size(img0,2);
    depth           = size(img0,3);
    all_imgs        = zeros(height,width,depth,n_img_msg,'uint8');
    for i=1:n_img_msg
            all_imgs(:,:,:,i) = readImage(compressed_imgs{i});
    end
    implay(all_imgs,32);
end


%%  Display plots
figure;
subplot(6,1,1); plot(t_odom, pose_x); ylabel('x')
subplot(6,1,2); plot(t_odom, pose_y); ylabel('y')
subplot(6,1,3); plot(t_imu, a_x); ylabel('a_x')
subplot(6,1,4); plot(t_imu, a_y); ylabel('a_y')
subplot(6,1,5); plot(t_imu, yaw); ylabel('yaw')
subplot(6,1,6); plot(t_imu, w_z); ylabel('w_x')

figure;
plot(pose_x, pose_y);
grid on; xlabel('x'); ylabel('y');

    
%%  Display sampling frequency
sprintf('avg imu sampling frequency \t: %f \t[Hz]', f_imu)
sprintf('avg odom sampling frequency \t: %f \t[Hz]', f_odom)
sprintf('avg img sampling frequency \t: %f \t[Hz]', f_img)