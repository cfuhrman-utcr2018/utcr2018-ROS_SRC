#!/usr/bin/env julia

using RobotOS
@rosimport std_msgs.msg: UInt8, Float32
rostypegen()
using std_msgs.msg

# Constants:
wheel_radius = 128.95e-3; # meters
gear_ratio = 1.0/12.0; # wheel turns/motor turns
max_rot = 2000.0; # maximum rpm of motors
PID = false

#=function loop(pub_obj)
    loop_rate = Rate(50.0)
    while ! is_shutdown()
        pwm = UInt8Msg(155)
        #pwm = convert(UInt8, pwm)
        publish(pub_obj, pwm)
        rossleep(loop_rate)
    end
end =#

function calc_pwm(target)
    target = convert(Float64, target) # make sure data types are OK
    # Calculate desired RPM
    rpm = abs(target)*60.0/(2.0*pi*wheel_radius*gear_ratio);
    if rpm > max_rot
        rpm = max_rot
    end
    # Calculte the PWM based off of RPM
    if target > 0.0
        pwm = (92.0/max_rot)*rpm+160.0;
    elseif target < 0.0
        pwm = 153.0-(92.0*rpm)/max_rot;
    else
        pwm = 155.0;
    end
end

#=function pid_pwm(target, effort)
    target = convert(Float64, target) # make sure data types are OK
    effort = convert(Float64, effort)
    speed = target + effort
    # Calculate desired RPM
    rpm = abs(speed)*60.0/(2.0*pi*wheel_radius*gear_ratio);
    if rpm > max_rot
        rpm = max_rot
    end
    # Calculte the PWM based off of RPM
    if speed > 0.0
        pwm = (92.0/max_rot)*rpm+160.0;
    elseif speed < 0.0
        pwm = 153.0-(92.0*rpm)/max_rot;
    else
        pwm = 155.0;
    end
end=#

function callback(msg::Float32Msg, pub_obj::Publisher{UInt8Msg})
    # Extract the data from rwheel_vtarget msg
    velocity = msg.data
    # Calculate the PWM
    pwm = calc_pwm(velocity)
    pwm = round(pwm)
    # Convert to ROS message
    pwm = UInt8Msg(pwm)
    #pwm = UInt8Msg(round(pwm)) # Must round to convert to intiger data type
    # Publish the data
    publish(pub_obj, pwm)
end

function main()
    init_node("speed_to_pwm")
    #r_pub = Publisher{UInt8Msg}("Duty_Cycle_Right", queue_size=10)
    #r_sub = Subscriber{Float32Msg}("rwheel_vtarget", callback, (r_pub,), queue_size=10)
    l_pub = Publisher{UInt8Msg}("Duty_Cycle_Left", queue_size=10)
    l_sub = Subscriber{Float32Msg}("lwheel_vtarget", callback, (l_pub,), queue_size=20)
    #=if PID == true
        r_effort =
        l_effort
    end=#
    spin()
end

if ! isinteractive()
    main()
end
