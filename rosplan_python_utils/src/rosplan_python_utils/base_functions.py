import rospy
import service_utils
import domain_utils
from diagnostic_msgs.msg import KeyValue
from rosplan_knowledge_msgs.srv import KnowledgeUpdateServiceArray, KnowledgeUpdateServiceArrayRequest
from rosplan_knowledge_msgs.msg import KnowledgeItem


def update(predicate, truth_value, add_action, remove_action):
    """update the knowledge base.
    :param predicate: The condition as a string taken from the PNP.
    :param truth_value: bool
    """
    srv_name = "/kcl_rosplan/update_knowledge_base_array"
    req = KnowledgeUpdateServiceArrayRequest()
    req.update_type = add_action if truth_value else remove_action
    predicate = [predicate] if not isinstance(predicate,list) else predicate
    for p in predicate:
        cond = p.split("__")
        rospy.loginfo("Updating %s %s" % (str(p), str(truth_value)))
        try:
            tp = domain_utils.get_predicate_details(cond[0]).predicate.typed_parameters
        except AttributeError as e:
            rospy.logwarn(e)
            return
        if len(tp) != len(cond[1:]):
            rospy.logerr("Fact '%s' should have %s parameters but has only %s as parsed from: '%s'" % (cond[0], len(tp), len(cond[1:]), p))
            return
        req.knowledge.append(KnowledgeItem(
            knowledge_type=KnowledgeItem.FACT,
            attribute_name=cond[0],
            values=[KeyValue(key=str(k.key), value=str(v)) for k,v in zip(tp, cond[1:])]
        ))

    while not rospy.is_shutdown():
        try:
            service_utils.call_service(
                srv_name,
                KnowledgeUpdateServiceArray,
                req
            )
        except rospy.ROSInterruptException:
            rospy.logerr("Communication with '%s' interrupted. Retrying." % srv_name)
            rospy.sleep(1.)
        else:
            return
