function [color_processed_image, depth_processed_image] ...
    = process_image(color_image, depth_image, Threshold_value)
% This function takes an image and returns a vertically flipped image that
% detects whites (especially lines). 
% Input: a MATLAB image, a threshold value (a whiteness value)
% Output: a binary image with a 1 for a pixel that contains white and a 0
%   for a pixel that does not
% Author: Connor Fuhrman

% Split the RBG image into 3 different channels
image_r = color_image(:,:,1); image_g = color_image(:,:,2); image_b = color_image(:,:,3);

gimage = rgb2gray(color_image); % Grayscale for Thresholding
TH = (gimage > Threshold_value);
BW = medfilt2(TH);
F = bwareaopen(BW,30);
image_r(F == 0) = 0;
image_g(F == 0) = 0;
image_b(F == 0) = 0;

color_processed_image(:,:,1) = image_r;
color_processed_image(:,:,2) = image_g;
color_processed_image(:,:,3) = image_b;

F = uint16(F);
depth_processed_image = F.*depth_image;
end