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
    % % File for testing with sample ROS PointCloud2 variable
    load ~/catkin_ws/src/matlab/'Saved Variables'/ROS_PointCloud2.mat
end

% Initiailze variables:
threshold_value = 0.7;
seq = 1;

if test == 0 
    % create subscriber to depth cloud:
    color_sub = rossubscriber('/camera/color/image_raw');
    %depth_sub = rossubscriber('/depth_image_meters');
    depth_sub = rossubscriber('camera/depth/image_rect_raw')
end

% create messages to pass the depth and rbg images
depth_msg = rosmessage('sensor_msgs/Image');
color_msg = rosmessage('sensor_msgs/Image');

depth_pub = rospublisher('/processed_depth','sensor_msgs/Image');
color_pub = rospublisher('/processed_color', 'sensor_msgs/Image'); 

color_msg.Encoding = 'rgb8';
depth_msg.Encoding = '16uc1';

color_msg.Header.FrameId = 'lines_camera';
depth_msg.Header.FrameId = 'lines_camera';

 while 1
    if test == 0 
        % Get data from ROS
        depth_ros = receive(depth_sub);
        color_ros = receive(color_sub);
    end
    
%while 1
    color = readImage(color_ros); depth = readImage(depth_ros);
    color = im2double(color); depth = im2double(depth);

    % Detect white lines and return an image showing only white lines:
    [lines_image, xyz_lines] = process_image(color, depth, threshold_value);
 
    % Convert the lines_image to RGB8 encoding
    lines_image = im2uint8(lines_image);
    % Convert xyz_lines into 16UC1 encoding
    xyz_lines = im2uint16(xyz_lines);
    
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
% end