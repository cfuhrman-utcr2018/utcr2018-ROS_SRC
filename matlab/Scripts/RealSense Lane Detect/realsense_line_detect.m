% This program connects to a ROS point cloud and to detect white lines
% through segmentation and color detection. It then plots the lines in real
% world units using the depth perception of the Intel Realsesnse Stereo
% vision camera
%
% This program utilizes the following MATLAB functions:
% - threshold.m --> detects the images and returns a binary 0 or 1 image
%   where the while lines are marked with a 1
% - 

clear, clc, close all
rosshutdown
rosinit

addpath(genpath('Functions')) % to use functions in different folder

% % File for testing with sample ROS PointCloud2 variable
% load ~/catkins_ws/src/matlab/'Saved Variables'/ROS_PointCloud2.mat

% Initiailze variables:
threshold_value = 0.7;

% create subscriber to depth cloud:
PointCloud_sub = rossubscriber('/camera/depth_registered/points');

% create messages to pass the depth and rbg images
depth = rosmessage('sensor_msgs/Image');
color = rosmessage('sensor_msgs/Image');

depth_pub = rospublisher('/processed_depth','sensor_msgs/Image');
color_pub = rospublisher('/processed_color', 'sensor_msgs/Image'); 

color.Encoding = 'rgb8';
depth.Encoding = 'rgb8';

color.Header.FrameId = 'lines_camera';
depth.Header.FrameId = 'lines_camera';

% while 1

    % Get data from ROS
    ptcloud_ros = receive(PointCloud_sub,10); % recieve the ROS cloud
    ptcloud_ros.PreserveStructureOnRead = true;

    im_ros = readRGB(ptcloud_ros); % create an rgb image from PointCloud
    depth_ros = readXYZ(ptcloud_ros); % create depth image from PointCloud

    % Detect white lines and return an image showing only white lines:
    lines_image = process_image(im_ros, threshold_value);
    % Create an XYZ matrix containing only lines
    xyz_lines = depth_ros.*lines_image;
    
    % Convert the lines_image to RGB8 encoding
    lines_image = im2uint8(lines_image);
    % Convert xyz_lines into RGB8 encoding
    xyz_lines = im2uint8(xyz_lines);
    
    time = rostime(now);
    color.Header.Stamp = time;
    depth.Header.Stamp = time;

    writeImage(depth, xyz_lines);
    writeImage(color, lines_image);
while 1
    send(depth_pub, depth);
    send(color_pub, color);
end
% end