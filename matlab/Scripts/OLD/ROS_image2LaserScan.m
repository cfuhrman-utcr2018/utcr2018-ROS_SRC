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
% im_ros = receive(im_sub,10); % recieve the ROS data
% im = readImage(im_ros); % decode the image

% Process the data:
im_th = threshold(im, 150);
% execure BirdsEyeTransform
[birdsEyeImage, range_max, spaceToOneSide] = birdseye(im_th);
x_max = 2*spaceToOneSide;

% image length and height
height = length(birdsEyeImage(:,1));
width = length(birdsEyeImage(1,:));
ranges = zeros(1,total_angles);
intensities = zeros(1,total_angles);

image_lidar = lidar_image_view(birdsEyeImage);
flipped_image = flipud(image_lidar);

% Create LaserScan
time = rostime('now'); % FOr the header, the time that the first scan was
% recieved by the device
for x = 1:width
    x_distance = x_max*(width/x);
    for y = 1:height
       y_distance = 0.0241*y+2.9697; % from calculations on image size
       % vs distance ahead of image
       pix = flipped_image(y,x);
       if pix ~= 0
           r = sqrt(x_distance^2+y_distance^2);
           t = atan2(y_distance,x_distance);
           eq = (angles - t) < line_scan.AngleIncrement;
           range_indx = find(eq);
           ranges(range_indx) = r;
           intensities(range_indx) = 100;
           break
       end
    end
end


line_scan.Header.Stamp = time;
line_scan.Ranges = ranges;
line_scan.Intensities = intensities;

% send the ROS message
lines_msg = line_scan;

send(lines_pub, lines_msg)

