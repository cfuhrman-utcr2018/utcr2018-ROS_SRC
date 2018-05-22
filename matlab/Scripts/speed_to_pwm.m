% This node takes a wheel velocity target and translates into PWM signals
% for the Victor 888 Motor Controllers. This node was written in MATLAB and
% translated to C for use in ROS with the differential_drive package

rosshutdown
clear, clc, close all
rosinit

% Constants:
wheel_radius = 128.95e-3; %meters
gear_ratio = 1/12; % wheel turns/motor turns
max_rot = 2000; % maximum rpm of motors
PID = true;

if PID == 1
    % Subscribe to both right and left motor commands (efforts)
    sub_left_motor_cmd = rossubscriber('lmotor_cmd');
    sub_right_motor_cmd = rossubscriber('rmotor_cmd');
end

% Subscribe to left and right velocity targets
sub_right_vel_target = rossubscriber('rwheel_vtarget');
sub_left_vel_target = rossubscriber('lwheel_vtarget');

% Create left and right PWM message
left_pwm_ros = rosmessage('std_msgs/UInt8');
right_pwm_ros = rosmessage('std_msgs/UInt8');

% Create left and right PWM publisher objects
pub_left_pwm = rospublisher('/Duty_Cycle_Left','std_msgs/UInt16');
pub_right_pwm = rospublisher('/Duty_Cycle_Right', 'std_msgs/UInt16');

while 1
    % Get the velocity target data
    right_vel_target = receive(sub_right_vel_target);
    left_vel_target = receive(sub_left_vel_target);
    % Extract the left target and convert to double
    left_target = left_vel_target.Data;
    left_target = double(left_target);
    % Extract the right target and convert to double
    right_target = right_vel_target.Data;
    right_target = double(right_target);
    
    if PID == 1
        % Get the effort data
        l_effort = receive(sub_left_motor_cmd);
        r_effort = receive(sub_right_motor_cmd);
        % Extract the left effort and convert to double
        left_command = l_effort.Data;
        left_command = double(left_command);
        % Extract the right command and convert to double
        right_command = r_effort.Data;
        right_command = double(right_command);
        % Offset the velocity target by the effort
        r_total_motor = right_target + right_command;
        l_total_motor = left_target + left_command;
        
        
    else
        % Convert from m/s to motor RPM
        left_rpm = abs(left_target)*60/(2*pi*wheel_radius*gear_ratio);
        right_rpm = abs(right_target)*60/(2*pi*wheel_radius*gear_ratio);
    end


    
    
    
    if left_rpm > max_rot
        left_rpm = max_rot;
    end
    if right_rpm > max_rot
        right_rpm = max_rot;
    end

    % Convert from rpm to PWM for right and left motor with cases for 
    % forward and reverse velocity commands. A 0 m/s command puts us in 
    % the middele of our motor controller curve for neutral
    if left_target > 0
        left_PWM = (92/max_rot)*left_rpm+160;
    elseif left_target < 0
        left_PWM = 153-(92*left_rpm)/max_rot;
    else
        left_PWM = 155;
    end

    if right_target > 0
        right_PWM = (92/max_rot)*right_rpm+160;
    elseif right_target < 0
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