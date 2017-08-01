import rospy


def call_service(srv_name, srv_type, req):
    while not rospy.is_shutdown():
        try:
            s = rospy.ServiceProxy(
                srv_name,
                srv_type
            )
            s.wait_for_service(timeout=1.)
        except rospy.ROSException, rospy.ServiceException:
            rospy.logwarn("Could not communicate with '%s' service. Retrying in 1 second." % srv_name)
            rospy.sleep(1.)
        else:
            return s(req)
