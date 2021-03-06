focalLength    = [length(im), length(im(:,1))]; % [fx, fy] in pixel units
principalPoint = [length(im)/2, length(im(:,1))/2]; % [cx, cy] optical center in pixel coordinates
imageSize      = [480, 752];           % [nrows, mcols]

camIntrinsics = cameraIntrinsics(focalLength, principalPoint, imageSize);

height = 1.71;    % mounting height in meters from the ground
pitch  = pi/6*180/pi;        % pitch of the camera in degrees

sensor = monoCamera(camIntrinsics, height, 'Pitch', pitch);

% Using vehicle coordinates, define area to transform
distAheadOfSensor = 10; % in meters, as previously specified in monoCamera height input
spaceToOneSide    = x(i);  % all other distance quantities are also in meters
bottomOffset      = 3;

outView   = [bottomOffset, distAheadOfSensor, -spaceToOneSide, spaceToOneSide]; % [xmin, xmax, ymin, ymax]
imageSize = [NaN, 250]; % output image width in pixels; height is chosen automatically to preserve units per pixel ratio

birdsEyeConfig = birdsEyeView(sensor, outView, imageSize);

birdsEyeImage = transformImage(birdsEyeConfig, F);
figure
imshow(birdsEyeImage)