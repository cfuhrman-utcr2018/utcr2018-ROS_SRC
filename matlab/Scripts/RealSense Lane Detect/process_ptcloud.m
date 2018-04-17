function ptcloud = process_ptcloud(RGB_image, XYZ_image, Processed_image)
% COMMENTS
RGB_image = RGB_image.*Processed_image; 
% Break 3-D matrix into 3 nxm matricies
X = XYZ_image(:,:,1); Y = XYZ_image(:,:,2); Z = XYZ_image(:,:,3);
X = X.*Processed_image; Y = Y.*Processed_image; Z = Z.*Processed_image;

xyz(:,:,1) = X; xyz(:,:,2) = Y; xyz(:,:,3) = Z;
ptcloud = pointCloud(xyz, 'Color', RGB_image);
% figure; pcshow(ptcloud)
end