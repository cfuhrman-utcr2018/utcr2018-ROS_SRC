% This program converts an ROS published image 
% into a LaserScan data type to be fed into the 
% Navigation Stack

%rosinit % Initialize ROS node
clear
load('sample_gazebo_image_lines.mat')

% LIDAR Parameters
angle_max = pi; angle_min = 0;
range_min = 0; %range_max = distAheadOfSensor from BirdsEyeTransform
total_angles = 700;
angles = linspace(angle_min,angle_max,total_angles);

% Create LaserScan Message and Populate Values
line_scan = rosmessage('sensor_msgs/LaserScan');
line_scan.Header.FrameId = 'hokuyo_link';
line_scan.AngleMax = angle_max; line_scan.AngleMin = angle_min;
line_scan.AngleIncrement = angles(2)-angles(1);
[lines_pub,lines_msg] = rospublisher('/lines','sensor_msgs/LaserScan');

% im_sub = rossubscriber('/camera/image_raw'); % Subscribe to cameara topic
% % Loop to here
% im_ros = receive(im_sub,10); % recieve the ROS data
% im = readImage(im_ros); % decode the image

% Process the data:
TH = (im > 150);
BW = medfilt2(TH);
F = bwareaopen(BW,30);
% convert to double
F = im2double(F);
% execure BirdsEyeTransform
[birdsEyeImage, range_max, spaceToOneSide] = birdseye(F);
x_max = 2*spaceToOneSide;

% image length and height
height = length(birdsEyeImage(:,1));
width = length(birdsEyeImage(1,:));
ranges = zeros(1,total_angles);
intensities = zeros(1,total_angles);

flipped_image = flipud(birdsEyeImage);

% Create LaserScan
for x = 1:width
    x_distance = x_max*(width/x);
    for y = 1:height
       y_distance = range_max*(height/y);
       pix = flipped_image(y,x);
       if pix ~= 0
           r = sqrt(x_distance^2+y_distance^2);
           t = atan(y_distance/x_distance);
           eq = (angles - t) == 0;
           range_indx = find(eq);
           ranges(range_indx) = r;
           intensities(range_indx) = 100;
           break
       end
    end
end
line_scan.Ranges = ranges;
line_scan.Intensities = intensities;

% send the ROS message
lines_msg = line_scan;

send(lines_pub, lines_msg)

