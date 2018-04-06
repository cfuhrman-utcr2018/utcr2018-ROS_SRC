% This program connects to a Gazebo simulation within ROS and pulls
% image data from the camera continuously. It impliments an image 
% processing algorithm to display all white.

clear, clc, close all

%rosinit

im_sub = rossubscriber('/camera/image_raw');
while 1
    im_ros = receive(im_sub,10);
    im = readImage(im_ros);
    %gray = rgb2gray(im);
    % imshow(im)
    %TH = (gray > 150);
    TH = (im > 100);
    BW = medfilt2(TH);
    F = bwareaopen(BW,30);
    imshow(im);
    drawnow;
end
