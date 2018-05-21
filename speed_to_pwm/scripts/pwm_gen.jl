#!/usr/bin/env julia

using RobotOS
@rosimport std_msgs.msg: UInt8
rostypegen()
using std_msgs.msg


function loop(pub_obj)
    loop_rate = Rate(50.0)
    while ! is_shutdown()
        pwm = UInt8Msg(155)
        #pwm = convert(UInt8, pwm)
        publish(pub_obj, pwm)
        rossleep(loop_rate)
    end
end

function main()
    init_node("speed_to_pwm")
    r_pub = Publisher{UInt8Msg}("Duty_Cycle_Right", queue_size=10)
    loop(r_pub)
end

if ! isinteractive()
    main()
end
