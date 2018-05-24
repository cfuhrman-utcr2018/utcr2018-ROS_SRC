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

test = false;

addpath(genpath('Functions')) % to use functions in different folder

if test == 1
    % File for testing with sample ROS PointCloud2 variable
    load ~/catkin_ws/src/matlab/'Saved Variables'/ROS_PointCloud2.mat
end

% Initiailze variables:
threshold_value = 0.7;
seq = 1;

if test == 0
    pointcloud_sub = rossubscriber('/camera/depth_registered/points'); 
end

% create messages to pass the depth and rbg images
depth_msg = rosmessage('sensor_msgs/Image');
color_msg = rosmessage('sensor_msgs/Image');

depth_pub = rospublisher('/processed_depth','sensor_msgs/Image');
color_pub = rospublisher('/processed_color', 'sensor_msgs/Image'); 

color_msg.Encoding = 'rgb8';
depth_msg.Encoding = '32FC1';

color_msg.Header.FrameId = 'lines_link';
depth_msg.Header.FrameId = 'lines_link';

 while 1
    if test == 0 
        % Get data from ROS
        ptcloud_ros = receive(pointcloud_sub);
    end
    ptcloud_ros.PreserveStructureOnRead = true;
    
    % Get the RGB and XYZ ptcloud data
    color = readRGB(ptcloud_ros); depth = readXYZ(ptcloud_ros);
    depth = depth(:,:,3); % Only need the z data
    

    % Detect white lines and return an image showing only white lines:
    [lines_image, xyz_lines] = process_image(color, depth, threshold_value);
 
    % Convert the lines_image to RGB8 encoding
    lines_image = im2uint8(lines_image);
    % Convert xyz_lines into 32FC1 encoding
    %xyz_lines = im2uint16(xyz_lines);
    
    time = rostime('now');
    color_msg.Header.Stamp = time;
    depth_msg.Header.Stamp = time;
    
    color_msg.Header.Seq = seq;
    depth_msg.Header.Seq = seq;

    writeImage(depth_msg, xyz_lines);
    writeImage(color_msg, lines_image);

    send(depth_pub, depth_msg);
    send(color_pub, color_msg);
    
    seq = seq + 1;
 end