    JointCommandMessage = JointCommand()
    JointCommandMessage.mode = 4
    if speed_ratio is None:
        speed_ratio = 0.3
        if girigiri_aiiiiii:
            speed_ratio = 0.8
    if accel_ratio is None:
        accel_ratio = 0.1
        if girigiri_aiiiiii:
            accel_ratio = 0.2
    plan = group.plan(target)
    rospy.sleep(1)
    step = []
    start_time = rospy.get_time()
    for point in plan.joint_trajectory.points:
        current_time = rospy.get_time() - start_time
        # step.append(point.positions)
        JointCommandMessage.positions = point.positions
        JointCommandMessage.velocity = point.velocities
        JointCommandMessage.acceleration = point.accelerations
        desire_time = point.time_from_start.to_secs()
        print desire_time
        # if current_time < plan.joint_trajectory.points[i].time_from_
        # rospy.publish(JointCommandMessage)
        
    # traj = MotionTrajectory(limb=limb)
    # wpt_opts = MotionWaypointOptions(max_joint_speed_ratio=speed_ratio,
    #                                  max_joint_accel=accel_ratio)
    # waypoint = MotionWaypoint(options=wpt_opts.to_msg(), limb=limb)
    
    exit()
    # for point in step:
    #     waypoint.set_joint_angles(joint_angles=point)
    #     print "each point is:" , point
    #     print " ++ way point as a list : ", traj.get_waypoint_joint_angles_as_list()
    #     # print " -- current waypoint is", waypoint._data
    #     traj.append_waypoint(waypoint.to_msg())
    # traj.send_trajectory(timeout=timeout)
    # group.stop()
    # group.clear_pose_targets()