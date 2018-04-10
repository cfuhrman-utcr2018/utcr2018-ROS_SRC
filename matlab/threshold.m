function im_th = threshold(image, Threshold_value)
TH = (image > Threshold_value);
BW = medfilt2(TH);
F = bwareaopen(BW,30);
% convert to double
im_th = im2double(F);
imshow(im_th)
end