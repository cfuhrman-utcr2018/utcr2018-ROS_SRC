% DO NOT USE

function [ranges, intensities] = lines_to_lidar(image, depth, angles)
% This function creates a ranges vector that accepts images extracted from
% the RGB and XYZ point cloud information. This then populates the distance
% to the identified points
% Input: a processed image in binary format that detects the white lines, an image
%   containing depth information
% Output: a ranges vector that contains a straight line distance from the
%   camerea to the line

[height, width] = size(lines_image);
robot_origin = width/2;

delta_angle = angles(2)-angles(1);
ranges = zeros(1, length(angles)); % initialize ranges
intensities = zeros(1, length(angles)); % initiailze intensities
% distances = depth.* image;

depth = flipud(depth); % so pixels match with already flipped image

for x = 1:width
    for y = 1:height
       pix = image(y,x);
       if pix ~= 0
           r = sqrt(depth(y,x,1)^2 + depth(y,x,2)^2 + depth(y,x,3)^2);
           if x > robot_origin
               x_robot = robot_origin - x;
               t = atan2(y,x_robot);
           elseif x < robot_origin
               x_robot = x - robot_origin;
               t = pi - atan2(y,x_robot);
           else
               t = pi/2;
           end
           eq = (angles - t) < delta_angle;
           range_indx = find(eq);
           ranges(range_indx) = r;
           intensities(range_indx) = 100;
           break
       end
    end
end

end