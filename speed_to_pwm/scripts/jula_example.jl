#!/usr/bin/env julia

using RobotOS
@rosimport geometry_msgs.msg: Point, Pose2D
rostypegen()
using geometry_msgs.msg

function callback(msg::Pose2D, pub_obj::Publisher{Point})
    pt_msg = Point(msg.x, msg.y, 0.0)
    publish(pub_obj, pt_msg)
end

function loop(pub_obj)
    loop_rate = Rate(5.0)
    while ! is_shutdown()
        npt = Point(rand(), rand(), 0.0)
        publish(pub_obj, npt)
        rossleep(loop_rate)
    end
end

function main()
    init_node("rosjl_example")
    pub = Publisher{Point}("pts", queue_size=10)
    sub = Subscriber{Pose2D}("pose", callback, (pub,), queue_size=10)
    loop(pub)
end

if ! isinteractive()
    main()
end
