% This program connecs to an already running ROS node 
% and reads an image from a Gazebo simulation 
clear, clc, close all

rosinit

im_sub = rossubscriber('/rrbot/camera1/image_raw')
im_ros = receive(im_sub,10)
im = readImage(im_ros);
imshow(im)

rosshutdown