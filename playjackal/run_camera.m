clear; close all; clc;

% Connect to the ROS master
% setenv('ROS_MASTER_URI','http://192.168.1.104:11311') % ROS IP
% setenv('ROS_IP','192.168.1.17') % MATLAB IP
% rosinit


% Image Display
sub_img = rossubscriber('/axis/image_raw/compressed');
img = receive(sub_img,10);

image = readImage(img);
figure; imshow(image)

% Point Cloud
sub_pcl = rossubscriber('/velodyne_points');
pcl = receive(sub_pcl,10);

figure; scatter3(pcl)
