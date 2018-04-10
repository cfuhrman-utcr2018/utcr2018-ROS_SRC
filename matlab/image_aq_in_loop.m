% Code takes many images and saves them in a loop
% Designed for camera calibration 


im_sub = rossubscriber('/camera/image_raw')

for i=1:150
    im_ros = receive(im_sub,1);
    im = readImage(im_ros);
    % imshow(im)
   imwrite(im,sprintf('%d.jpg',i))
   disp(i)
   pause(0.1)
end

