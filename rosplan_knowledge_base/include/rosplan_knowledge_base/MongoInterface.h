#include <iostream>
#include <vector>

#include<rosplan_knowledge_msgs/KnowledgeItem.h>

#include <mongo/client/dbclient.h>
#include <mongo/bson/bson.h>

#ifndef MONGOINTERFACE_H
#define MONGOINTERFACE_H


namespace KCL_rosplan {

typedef std::auto_ptr<mongo::DBClientCursor> db_cursor;

    class MongoInterface
    {
    private:
        std::string ns_k, ns_g;
    public:
        // DB params and client
        std::string dbName, dbCollection;
        mongo::DBClientConnection client;
        
        MongoInterface(std::string dbHost, std::string dbPort, std::string dbName, std::string dbCollection);
        
        //helper functions
        mongo::BSONObj knowledgeItemToBson(rosplan_knowledge_msgs::KnowledgeItem &ki);
        rosplan_knowledge_msgs::KnowledgeItem bsonToKnowledgeItem(mongo::BSONObj b);
        std::vector<rosplan_knowledge_msgs::KnowledgeItem> getEntries(std::string ns, mongo::Query query);
        void addEntry(std::string ns, rosplan_knowledge_msgs::KnowledgeItem &ki) { return addEntry(ns, knowledgeItemToBson(ki)); }
        void addEntry(std::string ns, mongo::BSONObj b);
        bool isEntry(std::string ns, rosplan_knowledge_msgs::KnowledgeItem &ki) { return isEntry(ns, knowledgeItemToBson(ki)); }
        bool isEntry(std::string ns, mongo::BSONObj b);
        void rmEntry(std::string ns, rosplan_knowledge_msgs::KnowledgeItem &ki) { return rmEntry(ns, knowledgeItemToBson(ki)); }
        void rmEntry(std::string ns, mongo::BSONObj b);
        
        // Convenience functions
        void addKnowledge(rosplan_knowledge_msgs::KnowledgeItem &ki) { return addKnowledge(knowledgeItemToBson(ki)); }
        void addKnowledge(mongo::BSONObj b) { return addEntry(ns_k, b); }
        void rmKnowledge(rosplan_knowledge_msgs::KnowledgeItem &ki) { return rmKnowledge(knowledgeItemToBson(ki)); }
        void rmKnowledge(mongo::BSONObj b) { return rmEntry(ns_k, b); }
        std::vector<rosplan_knowledge_msgs::KnowledgeItem> getKnowledge(mongo::Query query) { return getEntries(ns_k, query); }
        void addGoal(rosplan_knowledge_msgs::KnowledgeItem &ki) { return addGoal(knowledgeItemToBson(ki)); }
        void addGoal(mongo::BSONObj b) { return addEntry(ns_g, b); }
        bool isKnowledge(rosplan_knowledge_msgs::KnowledgeItem &ki) { return isKnowledge(knowledgeItemToBson(ki)); }
        bool isKnowledge(mongo::BSONObj b) { return isEntry(ns_k, b); }
        
        // Knowledge specific functions
        
        // Instances
        std::vector<rosplan_knowledge_msgs::KnowledgeItem> getInstances(std::string instance_type="");
        void removeInstances(std::string instance_type="");
        bool isInstance(rosplan_knowledge_msgs::KnowledgeItem &ki) { return isInstance(knowledgeItemToBson(ki)); }
        bool isInstance(mongo::BSONObj b) { return isKnowledge(b); }
        
        // Facts
        std::vector<rosplan_knowledge_msgs::KnowledgeItem> getFacts(std::string attribute_name="");
        void removeFacts(std::string attribute_name="");
        bool isFact(rosplan_knowledge_msgs::KnowledgeItem &ki) { return isFact(knowledgeItemToBson(ki)); }
        bool isFact(mongo::BSONObj b) { return isKnowledge(b); }
                
        // Functions
        std::vector<rosplan_knowledge_msgs::KnowledgeItem> getFunctions(std::string attribute_name="");
        void removeFunctions(std::string attribute_name="");
        bool isFunction(rosplan_knowledge_msgs::KnowledgeItem &ki) { return isFunction(knowledgeItemToBson(ki)); }
        bool isFunction(mongo::BSONObj b) { return isKnowledge(b); }
                
        // Goals
        std::vector<rosplan_knowledge_msgs::KnowledgeItem> getGoals(std::string attribute_name="");
        void removeGoals(std::string attribute_name="");
        bool isGoal(rosplan_knowledge_msgs::KnowledgeItem &ki) { return isGoal(knowledgeItemToBson(ki)); }
        bool isGoal(mongo::BSONObj b) { return isEntry(ns_g, b); }
    };

}

#endif // MONGOINTERFACE_H
