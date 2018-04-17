clear, close all

% Load appropriate camera parameters
load('Logitech_720pWebcam_Intrensics.mat')

%showExtrinsics(stereoParams); % to show camera properties from calibration
% session

Left_Cam = webcam(1); % Define Webcam for Left 
Right_Cam = webcam(2); % Define Webcam for Right

% Take images from each camera
Left_Image = snapshot(Left_Cam); 
Right_Image = snapshot(Right_Cam);

[frameLeftRect, frameRightRect] = ...
    rectifyStereoImages(Left_Image, Right_Image, stereoParams);

figure;
imshow(stereoAnaglyph(frameLeftRect, frameRightRect));
title('Rectified Video Frames');

LeftGray = rgb2gray(frameLeftRect); 
RightGray = rgb2gray(frameRightRect);

disparityMap = disparity(LeftGray, RightGray);
figure;
imshow(disparityMap, [0, 64]);
title('Disparity Map');
colormap jet
colorbar

points3D = reconstructScene(disparityMap, stereoParams);

% Convert to meters and create a pointCloud object
points3D = points3D ./ 100; % 100 for cm world units
ptCloud = pointCloud(points3D, 'Color', frameLeftRect);

% Create a streaming point cloud viewer
player3D = pcplayer([-3, 3], [-3, 3], [0, 8], 'VerticalAxis', 'y', ...
    'VerticalAxisDir', 'down');

% Visualize the point cloud
view(player3D, ptCloud);