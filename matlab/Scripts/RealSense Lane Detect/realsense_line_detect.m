% This program connects to a ROS point cloud and to detect white lines
% through segmentation and color detection. It then plots the lines in real
% world units using the depth perception of the Intel Realsesnse Stereo
% vision camera
%
% This program utilizes the following MATLAB functions:
% - threshold.m --> detects the images and returns a binary 0 or 1 image
%   where the while lines are marked with a 1
% - 

clear%, clc, close all
addpath(genpath('Functions')) % to use functions in different folder

% Initiailze variables:
threshold_value = 0.5;

% create subscriber to depth cloud:
depth_sub = rossubscriber('/camera/depth_registered/points');

% create published of processed data
lines_msg = rosmessage('sensor_msgs/PointCloud');
lines_msg.Header.FrameId = 'line_camera';
lines_pub = rospublisher('/lines','sensor_msgs/PointCloud');

% Loop to here!!


% Get data from ROS
depth_ros = receive(depth_sub,10); % recieve the ROS cloud
depth_ros.PreserveStructureOnRead = true;

im_ros = readRGB(depth_ros); % create an rgb image from PointCloud
depth_image = readXYZ(depth_ros); % create depth mapping from PointCloud

% Detect white lines and return an image showing only white lines:
lines_image = process_image(im_ros, threshold_value);

% create an RBG point cloud containing detected lines
% ptcloud = process_ptcloud(im_ros, depth_image, lines_image); % For 
    % creating a MATLAB ptcloud to visualize data within MATLAB
    
% Create an XYZ matrix containing only lines
xyz = depth_image.*lines_image;

% % Convert xyz from a MxNx3 matrix to a (M*N)x3 matrix as if the
% % PreserveStructureOnRead was set to FALSE
% xyz_Dx3 = mxnx3_to_dx3(xyz);
% 
% % Convert XYZ matrix into /sensor_msgs/PointCloud
% lines_msg = xyz_to_PointCloud(xyz_Dx3, lines_msg);
% lines_msg.Header.Stamp = rostime('now');

% create messages to pass the depth and rbg images
depth = rosmessage('sensor_msgs/Image');
color = rosmessage('sensor_msgs/Image');

depth_pub = rospublisher('/image_rect','sensor_msgs/Image');

depth.Encoding = '16UC1';
color.Encoding = 'rgb8';

writeImage(depth, uint16(xyz));
writeImage(color, uint8(lines_image));

send(depth_pub, depth);