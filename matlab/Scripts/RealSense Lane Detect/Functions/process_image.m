function processed_image = process_image(image, Threshold_value)
% This function takes an image and returns a vertically flipped image that
% detects whites (especially lines). 
% Input: a MATLAB image, a threshold value (a whiteness value)
% Output: a binary image with a 1 for a pixel that contains white and a 0
%   for a pixel that does not
% Author: Connor Fuhrman

image = rgb2gray(image); % Flipping for correct robot 
    % reference frame and grayscale for thresholding 
TH = (image > Threshold_value);
BW = medfilt2(TH);
F = bwareaopen(BW,30);
% convert to double
processed_image = im2double(F);
figure; imshow(processed_image)
end