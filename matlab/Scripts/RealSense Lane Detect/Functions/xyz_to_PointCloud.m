function msg = xyz_to_PointCloud(xyz,msg)
% This function from Robot Operating System The Comnplete Reference Volume
% 2, Robots Perception Through 3D Point Cloud Sensors Section 5.3 ROS
% Publisihing with MATLAB

xyzvalid = xyz (~isnan(xyz(:,1)),:); % removing undesirable NaN values 
    % from the variable xyz
PCL1mensage = rosmessage('geometry_msgs/Point32'); % creates a message
    % of type geometry_msgs/Point32. We must convert to this so that we can
    % translate our data into the PointCloud format
for i =1: size(xyzvalid ,1)
    % Loop that converts all points in the xyz matrix that are not NaNs
    % into the geometry_msgs/Point32 
    PCL1mensage(i).X = xyzvalid(i,1);
    PCL1mensage(i).Y = xyzvalid(i, 2);
    PCL1mensage(i).Z = xyzvalid(i, 3);
end

msg.Points = PCL1mensage;

end