focalLength    = [309.4362, 344.2161]; % [fx, fy] in pixel units
principalPoint = [318.9034, 257.5352]; % [cx, cy] optical center in pixel coordinates
imageSize      = [480, 752];           % [nrows, mcols]

camIntrinsics = cameraIntrinsics(focalLength, principalPoint, imageSize);

height = 1.71;    % mounting height in meters from the ground
pitch  = 0;        % pitch of the camera in degrees

sensor = monoCamera(camIntrinsics, height, 'Pitch', pitch);

% Using vehicle coordinates, define area to transform
distAheadOfSensor = 20; % in meters, as previously specified in monoCamera height input
spaceToOneSide    = 3;  % all other distance quantities are also in meters
bottomOffset      = 0;

outView   = [bottomOffset, distAheadOfSensor, -spaceToOneSide, spaceToOneSide]; % [xmin, xmax, ymin, ymax]
imageSize = [NaN, 250]; % output image width in pixels; height is chosen automatically to preserve units per pixel ratio

birdsEyeConfig = birdsEyeView(sensor, outView, imageSize);

birdsEyeImage = transformImage(birdsEyeConfig, im);
figure
imshow(birdsEyeImage)