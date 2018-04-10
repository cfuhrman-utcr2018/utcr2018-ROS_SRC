function [birdsEyeImage, distAheadOfSensor,spaceToOneSide ] = birdseye(image)
% focalLength    = [length(image), length(image(:,1))]; % [fx, fy] in pixel units
% principalPoint = [length(image)/2, length(image(:,1))/2]; % [cx, cy] optical center in pixel coordinates
% imageSize      = [480, 752];           % [nrows, mcols]
load('GigE Camera Params.mat');
focalLength = GigE_cameraParams.FocalLength;
principalPoint = GigE_cameraParams.PrincipalPoint;
imageSize = size(image);

camIntrinsics = cameraIntrinsics(focalLength, principalPoint, imageSize);

height = 1.71;    % mounting height in meters from the ground
pitch  = 42;        % pitch of the camera in degrees

sensor = monoCamera(camIntrinsics, height, 'Pitch', pitch);

% Using vehicle coordinates, define area to transform
distAheadOfSensor = 5; % in meters, as previously specified in monoCamera height input
spaceToOneSide    = 0.5;  % all other distance quantities are also in meters
bottomOffset      = 0.32;

outView   = [bottomOffset, distAheadOfSensor, -spaceToOneSide, spaceToOneSide]; % [xmin, xmax, ymin, ymax]
imageSize = [NaN, 250]; % output image width in pixels; height is chosen automatically to preserve units per pixel ratio

birdsEyeConfig = birdsEyeView(sensor, outView, imageSize);

[J,newOrigin] = undistortImage(image,GigE_cameraParams);

birdsEyeImage = transformImage(birdsEyeConfig, image);
end