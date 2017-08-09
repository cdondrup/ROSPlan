import base_functions
import service_utils
from rosplan_knowledge_msgs.srv import KnowledgeUpdateServiceArrayRequest
from std_srvs.srv import Empty, EmptyRequest


def update(predicate, truth_value):
    """update the knowledge base.
    :param predicate: The condition as a string taken from the PNP.
    :param truth_value: bool
    """
    base_functions.update(
        predicate=predicate,
        truth_value=truth_value,
        add_action=KnowledgeUpdateServiceArrayRequest.ADD_GOAL,
        remove_action=KnowledgeUpdateServiceArrayRequest.REMOVE_GOAL
    )


def add(predicate):
    update(predicate, True)


def remove(predicate):
    update(predicate, False)


def clear_all():
    service_utils.call_service("/kcl_rosplan/clear_goals", Empty, EmptyRequest())
