% This node takes a wheel velocity target and translates into PWM signals
% for the Victor 888 Motor Controllers. This node was written in MATLAB and
% translated to C for use in ROS with the differential_drive package
rosshutdown
clear, clc, close all
rosinit

% Constants:
wheel_radius = 128.95e-3; %meters
gear_ratio = 1/12; % wheel turns/motor turns
max_rot = 4320; % maximum rpm of motors


sub_left_motor_cmd = rossubscriber('lmotor_cmd');
sub_right_motor_cmd = rossubscriber('rmotor_cmd');

% Subscribe to the target velocity from Twist to Motors
sub_left_vel_target = rossubscriber('lwheel_vtarget');
sub_right_target = rossubscriber('rwheel_vtarget');

% Create left and right PWM message
left_pwm_ros = rosmessage('std_msgs/UInt16');
right_pwm_ros = rosmessage('std_msgs/UInt16');

% Create left and right PWM publisher objects
pub_left_pwm = rospublisher('/Duty_Cycle_Left','std_msgs/UInt16');
pub_right_pwm = rospublisher('/Duty_Cycle_Right', 'std_msgs/UInt16');

while 1
    lmotor_cmd = receive(sub_left_motor_cmd);
    rmotor_cmd = receive(sub_right_motor_cmd);
    
    right_target_ros = receive(sub_right_target);
    left_target_ros = receive(sub_left_vel_target);
    
    left_target = left_target_ros.Data;
    left_target = double(left_target);
    
    right_target = right_target_ros.Data;
    right_target = double(right_target);

    left_motor_cmd = lmotor_cmd.Data; % This is the speed target for the 
        % left motor in m/s to be translated into PWM signals to the motor 
        % controllers
    left_motor_cmd = double(left_motor_cmd);

    
    right_motor_cmd = rmotor_cmd.Data;
    right_motor_cmd = double(right_motor_cmd);
    
    corrected_l = left_target+left_motor_cmd;
    corrected_r = right_target+right_motor_cmd;
    
    % Convert from m/s to motor RPM
    left_rpm = abs(corrected_l)*60/(2*pi*wheel_radius*gear_ratio);
    right_rpm = abs(corrected_r)*60/(2*pi*wheel_radius*gear_ratio);
    
    if left_rpm > max_rot
        left_rpm = max_rot;
    end
    if right_rpm > max_rot
        right_rpm = max_rot;
    end

    % Convert from rpm to PWM for right and left motor with cases for 
    % forward and reverse velocity commands. A 0 m/s command puts us in 
    % the middele of our motor controller curve for neutral
    if corrected_l > 0
        left_PWM = (92/max_rot)*left_rpm+160;
    elseif corrected_l < 0
        left_PWM = 153-(92*left_rpm)/max_rot;
    else
        left_PWM = 155;
    end

    if corrected_r > 0
        right_PWM = (92/max_rot)*right_rpm+160;
    elseif corrected_r < 0
        right_PWM = 153-(152*right_rpm)/max_rot;    
    else
        right_PWM = 155;
    end

    % Assign PWM to ROS Message 
    left_pwm_ros.Data = uint8(left_PWM);
    right_pwm_ros.Data = uint8(right_PWM);

    % Send the published data
    send(pub_left_pwm, left_pwm_ros);
    send(pub_right_pwm, right_pwm_ros);
end