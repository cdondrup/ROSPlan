import rospy
import service_utils
from rosplan_knowledge_msgs.srv import GetDomainPredicateDetailsService


def get_predicate_details(name):
    srv_name = "/kcl_rosplan/get_domain_predicate_details"
    while not rospy.is_shutdown():
        try:
            return service_utils.call_service(
                srv_name,
                GetDomainPredicateDetailsService,
                name
            )
        except rospy.ROSInterruptException:
            rospy.logerr("Communication with '%s' interrupted. Retrying." % srv_name)
            rospy.sleep(1.)
