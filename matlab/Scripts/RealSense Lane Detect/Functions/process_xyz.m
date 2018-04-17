function xyz = process_xyz(XYZ_image, Processed_image)
% This function takes a depth image and outputs the portions of the image
% that were identified through a lane detection algorithm to pick out the
% white lines only

X = XYZ_image(:,:,1); Y = XYZ_image(:,:,2); Z = XYZ_image(:,:,3);
X = X.*Processed_image; Y = Y.*Processed_image; Z = Z.*Processed_image;

xyz(:,:,1) = X; xyz(:,:,2) = Y; xyz(:,:,3) = Z;
end