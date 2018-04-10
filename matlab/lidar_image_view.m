function image_lidar = lidar_image_view(image)
% This function takes any image and only displays the contet that would
% be in the line of sight to a planar LIDAR system. 
% Input - a thresholded image with values either zero or nonzero
% Output - an image with a value of 1 for every pixel containing
% information and a zero for the rest of that column. 
    image = flipud(image);
    for x = 1:length(image(1,:))
        for y = 1:length(image(:,1))
            if image(y,x) ~= 0
                image(y,x) = 1;
                image(y+1:end,x) = 0;
                break
            end
        end
    end
    image_lidar = flipud(image);
end