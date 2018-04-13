clear, clc, close all

Left_Cam = webcam(1); % Define Webcam for Left 
Right_Cam = webcam(2); % Define Webcam for Right

% preview(Left_Cam);
% preview(Right_Cam);

Left_Path = '/home/pablocore/catkin_ws/src/matlab/Camera Calibration Photos/Stereo Calibratinon/Left Camera';
Right_Path = '/home/pablocore/catkin_ws/src/matlab/Camera Calibration Photos/Stereo Calibratinon/Right Camera';

nametemplate_Left = 'Left image_%04d.png';  %name pattern
nametemplate_Right = 'Right image_%04d.png';  %name pattern

im_num = 1;

for loops = 1:150
    % Aquire Image of Left and Right
    Left_Image = snapshot(Left_Cam); 
    Right_Image = snapshot(Right_Cam);
    
%     Left_Image = imrotate(Left_Image, 90);
%     Right_Image = imrotate(Right_Image, -90);
    
    subplot(1,2,1); imshow(Left_Image)
    subplot(1,2,2); imshow(Right_Image)


    Left_File = sprintf(nametemplate_Left, im_num);  %create Left Camera filename
    LeftName = fullfile(Left_Path, Left_File);  %folder and all

    Right_File = sprintf(nametemplate_Right, im_num); % create Right Camera filename
    RightName = fullfile(Right_Path, Right_File); %folder and all

    % Save the right and left images
    imwrite(Left_Image, LeftName);
    imwrite(Right_Image, RightName);
    
    im_num = im_num + 1;
    
    X = sprintf('I have saved file numer %04d',im_num); disp(X)
    
    %pause(0.1);
    
end

stereoCameraCalibrator