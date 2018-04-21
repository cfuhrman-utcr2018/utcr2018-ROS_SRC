% This node takes a wheel velocity target and translates into PWM signals
% for the Victor 888 Motor Controllers. This node was written in MATLAB and
% translated to C for use in ROS with the differential_drive package

% Constants:
wheel_radius = 128.95e-3; %meters
gear_ratio = 1/12; % wheel turns/motor turns
max_rot = 4320; % maximum rpm of motors

sub_left_target = rossubscriber('lmotor_cmd');
sub_right_target = rossubscriber('rmotor_cmd');

% Create left and right PWM message
left_pwm_ros = rosmessage('std_msgs/UInt16');
right_pwm_ros = rosmessage('std_msgs/UInt16');

% Create left and right PWM publisher objects
pub_left_pwm = rospublisher('/Duty_Cycle_Left','std_msgs/UInt16');
pub_right_pwm = rospublisher('/Duty_Cycle_Right', 'std_msgs/UInt16');

while 1
    left_target_ros = receive(sub_left_target);
    right_target_ros = receive(sub_right_target);

    left_target = left_target_ros.Data; % This is the speed target for the 
        % left motor in m/s to be translated into PWM signals to the motor 
        % controllers
    right_target = right_target_ros.Data;

    % Convert L and R target to double data type
    left_target = double(left_target);
    right_target = double(right_target);
    
    % Convert from m/s to motor RPM
    left_rpm = left_target*60/(2*pi*wheel_radius*gear_ratio);
    right_rpm = right_target*60/(2*pi*wheel_radius*gear_ratio);

    % Convert from rpm to PWM for right and left motor with cases for 
    % forward and reverse velocity commands. A 0 m/s command puts us in 
    % the middele of our motor controller curve for neutral
    if left_target > 0
        left_PWM = (92/max_rot)*left_rpm+163;
    elseif left_target < 0
        left_PWM = (152/max_rot)*left_rpm+1;
    else
        left_PWM = 155;
    end

    if right_target > 0
        right_PWM = (92/max_rot)*right_rpm+163;
    elseif right_target < 0
        right_PWM = (152/max_rot)*right_rpm+1;
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