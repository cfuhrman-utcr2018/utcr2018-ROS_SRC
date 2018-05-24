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

% Initiailze variables:
threshold_value = 0.7;
seq = 1;

color_sub = rossubscriber('/camera/color/image_raw'); 
depth_sub = rossubscriber('/camera/depth/image_rect_raw');

% create messages to pass the depth image
depth_msg = rosmessage('sensor_msgs/Image');
depth_pub = rospublisher('/processed_depth','sensor_msgs/Image');

depth_msg.Encoding = '32FC1';

depth_msg.Header.FrameId = 'lines_link';

 while 1
     color_ros = receive(color_sub);
     depth_ros = receive(depth_sub);
     
     color = readImage(color_ros);
     depth = readImage(depth_ros);
     
    % Detect white lines and return an image showing only white lines:
    [lines_image, xyz_lines] = process_image(color, depth, threshold_value);
 
    time = rostime('now');
    depth_msg.Header.Stamp = time;
    depth_msg.Header.Seq = seq;
    writeImage(depth_msg, xyz_lines);
    send(depth_pub, depth_msg);    
    seq = seq + 1;
 end