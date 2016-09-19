#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>

#include <std_srvs/Empty.h>
#include <diagnostic_msgs/KeyValue.h>

#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateServiceArray.h"
#include "rosplan_knowledge_msgs/KnowledgeQueryService.h"

#include "rosplan_knowledge_msgs/GetDomainTypeService.h"
#include "rosplan_knowledge_msgs/GetDomainAttributeService.h"
#include "rosplan_knowledge_msgs/GetDomainOperatorService.h"
#include "rosplan_knowledge_msgs/GetDomainOperatorDetailsService.h"
#include "rosplan_knowledge_msgs/GetDomainPredicateDetailsService.h"
#include "rosplan_knowledge_msgs/DomainFormula.h"

#include "rosplan_knowledge_msgs/GetAttributeService.h"
#include "rosplan_knowledge_msgs/GetInstanceService.h"
#include "rosplan_knowledge_msgs/KnowledgeItem.h"

#include "rosplan_knowledge_msgs/Notification.h"
#include "rosplan_knowledge_msgs/Filter.h"

#include "KnowledgeComparitor.h"
#include "PlanFilter.h"
#include "DomainParser.h"
#include "VALVisitorOperator.h"
#include "VALVisitorPredicate.h"

#include <mongo/client/dbclient.h>
#include <mongo/bson/bson.h>

#ifndef KCL_knowledgebase
#define KCL_knowledgebase

namespace KCL_rosplan {

#define INSTANCES ".instances"
#define FACTS ".facts"
#define FUNCTIONS ".functions"

typedef std::auto_ptr<mongo::DBClientCursor> db_cursor;

	class KnowledgeBase
	{
	private:

        // DB params
        std::string dbName, dbCollection;
        mongo::DBClientConnection client;
        
		// visit controllers for ROS message packing
		VALVisitorOperator op_visitor;
		VALVisitorPredicate pred_visitor;

		// adding and removing items to and from the knowledge base
		void addKnowledge(rosplan_knowledge_msgs::KnowledgeItem &msg);
		void addMissionGoal(rosplan_knowledge_msgs::KnowledgeItem &msg);
		void removeKnowledge(rosplan_knowledge_msgs::KnowledgeItem &msg);
		void removeMissionGoal(rosplan_knowledge_msgs::KnowledgeItem &msg);

        // Mongo helper functions
        mongo::BSONObj knowledgeItemToBson(rosplan_knowledge_msgs::KnowledgeItem &ki);
        rosplan_knowledge_msgs::KnowledgeItem bsonToKnowledgeItem(mongo::BSONObj b);
        //bool containsKnowledge(std::string ns, rosplan_knowledge_msgs::KnowledgeItem &ki);
        std::vector<rosplan_knowledge_msgs::KnowledgeItem> getEntries(std::string ns, mongo::Query query);
        bool isEntry(std::string ns, mongo::BSONObj b);

	public:
        
        KnowledgeBase(std::string dbHost, std::string dbPort, std::string dbName, std::string dbCollection);

		// domain
		DomainParser domain_parser;

		// model
		std::map<std::string, std::vector<std::string> > model_instances;
        std::vector<rosplan_knowledge_msgs::KnowledgeItem> getInstances(std::string instance_type="");
        bool isInstance(rosplan_knowledge_msgs::KnowledgeItem &ki);
        bool isInstance(mongo::BSONObj b);
        std::vector<rosplan_knowledge_msgs::KnowledgeItem> model_facts;
        std::vector<rosplan_knowledge_msgs::KnowledgeItem> getFacts(std::string attribute_name="");
        bool isFact(rosplan_knowledge_msgs::KnowledgeItem &ki);
        bool isFact(mongo::BSONObj b);
		std::vector<rosplan_knowledge_msgs::KnowledgeItem> model_functions;
        std::vector<rosplan_knowledge_msgs::KnowledgeItem> getFunctions(std::string attribute_name="");
        bool isFunction(rosplan_knowledge_msgs::KnowledgeItem &ki);
        bool isFunction(mongo::BSONObj b);
		std::vector<rosplan_knowledge_msgs::KnowledgeItem> model_goals;

		// plan and mission filter
		PlanFilter plan_filter;

		/* fetching the domain */
		bool getTyes(rosplan_knowledge_msgs::GetDomainTypeService::Request  &req, rosplan_knowledge_msgs::GetDomainTypeService::Response &res);		
		bool getPredicates(rosplan_knowledge_msgs::GetDomainAttributeService::Request  &req, rosplan_knowledge_msgs::GetDomainAttributeService::Response &res);
		bool getFunctions(rosplan_knowledge_msgs::GetDomainAttributeService::Request  &req, rosplan_knowledge_msgs::GetDomainAttributeService::Response &res);
		bool getOperators(rosplan_knowledge_msgs::GetDomainOperatorService::Request  &req, rosplan_knowledge_msgs::GetDomainOperatorService::Response &res);
		bool getOperatorDetails(rosplan_knowledge_msgs::GetDomainOperatorDetailsService::Request  &req, rosplan_knowledge_msgs::GetDomainOperatorDetailsService::Response &res);
		bool getPredicateDetails(rosplan_knowledge_msgs::GetDomainPredicateDetailsService::Request  &req, rosplan_knowledge_msgs::GetDomainPredicateDetailsService::Response &res);

		// checking the model
		bool queryKnowledge(rosplan_knowledge_msgs::KnowledgeQueryService::Request  &req, rosplan_knowledge_msgs::KnowledgeQueryService::Response &res);

		// fetching the model
		bool getCurrentInstances(rosplan_knowledge_msgs::GetInstanceService::Request  &req, rosplan_knowledge_msgs::GetInstanceService::Response &res);
		bool getCurrentKnowledge(rosplan_knowledge_msgs::GetAttributeService::Request  &req, rosplan_knowledge_msgs::GetAttributeService::Response &res);
		bool getCurrentGoals(rosplan_knowledge_msgs::GetAttributeService::Request  &req, rosplan_knowledge_msgs::GetAttributeService::Response &res);

		// adding and removing items to and from the knowledge base
		bool updateKnowledge(rosplan_knowledge_msgs::KnowledgeUpdateService::Request  &req, rosplan_knowledge_msgs::KnowledgeUpdateService::Response &res);
		bool updateKnowledgeArray(rosplan_knowledge_msgs::KnowledgeUpdateServiceArray::Request &req, rosplan_knowledge_msgs::KnowledgeUpdateServiceArray::Response &res);
		bool clearKnowledge(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);
	};
}
#endif
