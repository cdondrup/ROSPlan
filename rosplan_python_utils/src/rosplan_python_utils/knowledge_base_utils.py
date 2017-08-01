import rospy
import service_utils
import domain_utils
from diagnostic_msgs.msg import KeyValue
from rosplan_knowledge_msgs.srv import KnowledgeQueryService, KnowledgeQueryServiceRequest
from rosplan_knowledge_msgs.srv import KnowledgeUpdateServiceArray, KnowledgeUpdateServiceArrayRequest
from rosplan_knowledge_msgs.msg import KnowledgeItem


def query(predicate):
    """query the knowledge base.
    :param predicate: The condition as a string taken from the PNP.
    :return (int) -1 (unknown), 0 (false), or 1 (true)
    """
    cond = predicate.split("__")
    srv_name = "/kcl_rosplan/query_knowledge_base"
    try:
        tp = domain_utils.get_predicate_details(cond[0]).predicate.typed_parameters
    except AttributeError as e:
        rospy.logwarn(e)
        return 0
    if len(tp) != len(cond[1:]):
        rospy.logerr("Fact '%s' should have %s parameters but has only %s as parsed from: '%s'" % (
            cond[0], len(tp), len(cond[1:])))
        return 0
    req = KnowledgeQueryServiceRequest()
    req.knowledge.append(
        KnowledgeItem(
            knowledge_type=KnowledgeItem.FACT,
            attribute_name=cond[0],
            values=[KeyValue(key=str(k.key), value=str(v)) for k, v in zip(tp, cond[1:])]
        )
    )
    while not rospy.is_shutdown():
        try:
            r = service_utils.call_service(
                srv_name,
                KnowledgeQueryService,
                req
            )
        except rospy.ROSInterruptException:
            rospy.logerr("Communication with '%s' interrupted. Retrying." % srv_name)
            rospy.sleep(1.)
        else:
            return 1 if r.all_true else 0


def update(predicate, truth_value):
    """update the knowledge base.
    :param predicate: The condition as a string taken from the PNP.
    :param truth_value: (int) -1 (unknown), 0 (false), 1 (true)
    """
    srv_name = "/kcl_rosplan/update_knowledge_base_array"
    req = KnowledgeUpdateServiceArrayRequest()
    req.update_type = req.ADD_KNOWLEDGE if truth_value else req.REMOVE_KNOWLEDGE
    predicate = [predicate] if not isinstance(predicate,list) else predicate
    for p in predicate:
        cond = p.split("__")
        rospy.loginfo("Updating %s %s" % (str(p), str(truth_value)))
        tp = domain_utils.get_predicate_details(cond[0]).predicate.typed_parameters
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
