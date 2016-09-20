#include "rosplan_knowledge_base/KnowledgeBasePersistent.h"

namespace KCL_rosplan {

    KnowledgeBasePersistent::KnowledgeBasePersistent(std::string dbHost, std::string dbPort, std::string dbName, std::string dbCollection) {
        mongo_interface = new MongoInterface(dbHost, dbPort, dbName, dbCollection);
    }
    
    KnowledgeBasePersistent::~KnowledgeBasePersistent() {
        delete mongo_interface;
    }

	/*-----------------*/
	/* knowledge query */
	/*-----------------*/

	bool KnowledgeBasePersistent::queryKnowledge(rosplan_knowledge_msgs::KnowledgeQueryService::Request  &req, rosplan_knowledge_msgs::KnowledgeQueryService::Response &res) {

		res.all_true = true;
		std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator iit;
		for(iit = req.knowledge.begin(); iit!=req.knowledge.end(); iit++) {

			bool present = false;
			if(iit->knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::INSTANCE) {

				// check if instance exists
				std::vector<std::string>::iterator sit;
				sit = find(model_instances[iit->instance_type].begin(), model_instances[iit->instance_type].end(), iit->instance_name);
				present = (sit!=model_instances[iit->instance_type].end());
				
			} else if(iit->knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::FUNCTION) {

				// check if function exists; TODO inequalities
				std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator pit;
				for(pit=model_functions.begin(); pit!=model_functions.end(); pit++) {
					if(KnowledgeComparitor::containsKnowledge(*iit, *pit)) {
						present = true;
						pit = model_functions.end();
					}
				}
			} else if(iit->knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::FACT) {

				// check if fact is true
				std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator pit;
				for(pit=model_facts.begin(); pit!=model_facts.end(); pit++) {
					if(KnowledgeComparitor::containsKnowledge(*iit, *pit)) {
						present = true;
						break;
					}
				}
			}

			if(!present) {
				res.all_true = false;
				res.results.push_back(false);
				res.false_knowledge.push_back(*iit);
			} else {
				res.results.push_back(true);
			}
		}

		return true;
	}

	/*----------------*/
	/* removing items */
	/*----------------*/

	void KnowledgeBasePersistent::removeKnowledge(rosplan_knowledge_msgs::KnowledgeItem &msg) {

		if(msg.knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::INSTANCE) {		

			// search for instance
			std::vector<std::string>::iterator iit;
			for(iit = model_instances[msg.instance_type].begin(); iit!=model_instances[msg.instance_type].end(); iit++) {

				std::string name = *iit;

				if(name.compare(msg.instance_name)==0 || msg.instance_name.compare("")==0) {

					// remove instance from knowledge base
					ROS_INFO("KCL: (KB) Removing instance (%s, %s)", msg.instance_type.c_str(), (msg.instance_name.compare("")==0) ? "ALL" : msg.instance_name.c_str());
					iit = model_instances[msg.instance_type].erase(iit);
					if(iit!=model_instances[msg.instance_type].begin()) iit--;
					plan_filter.checkFilters(msg, false);

					// remove affected domain attributes
					std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator pit;
					for(pit=model_facts.begin(); pit!=model_facts.end(); pit++) {
						if(KnowledgeComparitor::containsInstance(*pit, name)) {
							ROS_INFO("KCL: (KB) Removing domain attribute (%s)", pit->attribute_name.c_str());
							plan_filter.checkFilters(*pit, false);
							pit = model_facts.erase(pit);
							if(pit!=model_facts.begin()) pit--;
							if(pit==model_facts.end()) break;
						}
					}

					// finish
					if(iit==model_instances[msg.instance_type].end()) break;
				}
			}

		} else if(msg.knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::FUNCTION) {

			// remove domain attribute (function) from knowledge base
			std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator pit;
			for(pit=model_functions.begin(); pit!=model_functions.end(); pit++) {
				if(KnowledgeComparitor::containsKnowledge(msg, *pit)) {
					ROS_INFO("KCL: (KB) Removing domain attribute (%s)", msg.attribute_name.c_str());
					plan_filter.checkFilters(msg, false);
					pit = model_functions.erase(pit);
					if(pit!=model_functions.begin()) pit--;
					if(pit==model_functions.end()) break;
				}
			}

		} else if(msg.knowledge_type == rosplan_knowledge_msgs::KnowledgeItem::FACT) {

			// remove domain attribute (predicate) from knowledge base
			std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator pit;
			for(pit=model_facts.begin(); pit!=model_facts.end(); pit++) {
				if(KnowledgeComparitor::containsKnowledge(msg, *pit)) {
					ROS_INFO("KCL: (KB) Removing domain attribute (%s)", msg.attribute_name.c_str());
					plan_filter.checkFilters(msg, false);
					pit = model_facts.erase(pit);
					if(pit!=model_facts.begin()) pit--;
					if(pit==model_facts.end()) break;
				}
			}
		}
	}

	/**
	 * remove everything
	 */
	bool KnowledgeBasePersistent::clearKnowledge(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res) {

		ROS_INFO("KCL: (KB) Removing whole model");

		// model
		mongo_interface->rmKnowledge(mongo::BSONObj());
		mongo_interface->removeGoals();
	}

	/**
	 * remove mission goal
	 */
	void KnowledgeBasePersistent::removeMissionGoal(rosplan_knowledge_msgs::KnowledgeItem &msg) {

		bool changed = false;
		std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator git;
		for(git=model_goals.begin(); git!=model_goals.end(); git++) {
			if(KnowledgeComparitor::containsKnowledge(msg, *git)) {
				ROS_INFO("KCL: (KB) Removing goal (%s)", msg.attribute_name.c_str());
				git = model_goals.erase(git);
				if(git!=model_goals.begin()) git--;
				if(git==model_goals.end()) break;
			}
		}

		if(changed) {			
			rosplan_knowledge_msgs::Notification notMsg;
			notMsg.function = rosplan_knowledge_msgs::Notification::REMOVED;
			notMsg.knowledge_item = msg;
			plan_filter.notification_publisher.publish(notMsg);
		}
	}

	/*--------------*/
	/* adding items */
	/*--------------*/

	/*
	 * add an instance, domain predicate, or function to the knowledge base
	 */
	void KnowledgeBasePersistent::addKnowledge(rosplan_knowledge_msgs::KnowledgeItem &msg) {
        if(!mongo_interface->isKnowledge(msg)) {
            ROS_INFO("KCL: (KB) Adding knowledge");
            mongo_interface->addKnowledge(msg);
            plan_filter.checkFilters(msg, true);
        } else {
            ROS_INFO("KCL: (KB) Knowledge already in KB");
        }
	}

	/*
	 * add mission goal to knowledge base
	 */
	void KnowledgeBasePersistent::addMissionGoal(rosplan_knowledge_msgs::KnowledgeItem &msg) {
        if(!mongo_interface->isGoal(msg)) {
            ROS_INFO("KCL: (KB) Adding goal");
            mongo_interface->addGoal(msg);
        } else {
            ROS_INFO("KCL: (KB) Goal already in KB");
        }
	}

	/*----------------*/
	/* fetching items */
	/*----------------*/

	bool KnowledgeBasePersistent::getCurrentInstances(rosplan_knowledge_msgs::GetInstanceService::Request  &req, rosplan_knowledge_msgs::GetInstanceService::Response &res) {
	
		// fetch the instances of the correct type
        std::vector<rosplan_knowledge_msgs::KnowledgeItem> i = mongo_interface->getInstances(req.type_name);
        std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator iit;
        for(iit=i.begin(); iit != i.end(); iit++) {
            res.instances.push_back((*iit).instance_name);
        }
		return true;
	}

	bool KnowledgeBasePersistent::getCurrentKnowledge(rosplan_knowledge_msgs::GetAttributeService::Request  &req, rosplan_knowledge_msgs::GetAttributeService::Response &res) {
        mongo::BSONObjBuilder b;
        b.append("$or", BSON_ARRAY(
                     BSON("knowledge_type" << rosplan_knowledge_msgs::KnowledgeItem::FACT) << 
                     BSON("knowledge_type" << rosplan_knowledge_msgs::KnowledgeItem::FUNCTION)
                 ));
        if(req.predicate_name.compare("")!=0) b.append("attribute_name", req.predicate_name);

        std::vector<rosplan_knowledge_msgs::KnowledgeItem> i = mongo_interface->getKnowledge(b.obj());
        std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator iit;
        for(iit=i.begin(); iit != i.end(); iit++) {
            res.attributes.push_back(*iit);
        }
		return true;
	}

	bool KnowledgeBasePersistent::getCurrentGoals(rosplan_knowledge_msgs::GetAttributeService::Request  &req, rosplan_knowledge_msgs::GetAttributeService::Response &res) {

        std::vector<rosplan_knowledge_msgs::KnowledgeItem> i = mongo_interface->getGoals(req.predicate_name);
        std::vector<rosplan_knowledge_msgs::KnowledgeItem>::iterator iit;
        for(iit=i.begin(); iit != i.end(); iit++) {
            res.attributes.push_back(*iit);
        }
		return true;
	}

} // close namespace
