#include "rosplan_knowledge_base/MongoInterface.h"

namespace KCL_rosplan {
    MongoInterface::MongoInterface(std::string dbHost, std::string dbPort, std::string dbName, std::string dbCollection)
        : dbName(dbName), dbCollection(dbCollection)
    {
        ns_k = dbName+".knowledge";
        ns_g = dbName+".goals";
        mongo::client::initialize();
        try {
            std::cout << "DB: " << dbHost << ":" << dbPort << std::endl;
            client.connect(dbHost+":"+dbPort);
            std::cout << "connected ok" << std::endl;
        } catch( const mongo::DBException &e ) {
            std::cout << "caught " << e.what() << std::endl;
        }
    }
    
    // Helper functions
    mongo::BSONObj MongoInterface::knowledgeItemToBson(rosplan_knowledge_msgs::KnowledgeItem &ki) {
        mongo::BSONArrayBuilder ab;
        for(int vit = 0; vit < ki.values.size(); vit++) {
            ab.append(BSON("key" << ki.values[vit].key << "value" << ki.values[vit].value));
        }
        mongo::BSONObj p = BSON(
                    "instance_type" << ki.instance_type 
                    << "instance_name" << ki.instance_name
                    << "attribute_name" << ki.attribute_name
                    << "function_value" << ki.function_value
                    << "is_negative" << ki.is_negative
                    << "knowledge_type" << ki.knowledge_type
                    << "values" << ab.obj()
                    );
        return p;
    }
    
    rosplan_knowledge_msgs::KnowledgeItem MongoInterface::bsonToKnowledgeItem(mongo::BSONObj b) {
        rosplan_knowledge_msgs::KnowledgeItem ki;
        ki.instance_type = b.getField("instance_type").String();
        ki.instance_name = b.getField("instance_name").String();
        ki.attribute_name = b.getField("attribute_name").String();
        ki.function_value = b.getField("function_value").Double();
        ki.is_negative = (bool)b.getField("is_negative").Int();
        ki.knowledge_type = b.getField("knowledge_type").Int();
        std::vector<mongo::BSONElement> a;
        b.getObjectField("values").elems(a);
        for(int i = 0; i < a.size(); i++) {
            diagnostic_msgs::KeyValue kv;
            kv.key = a[i].embeddedObject().getField("key").String();
            kv.value = a[i].embeddedObject().getField("value").String();
            ki.values.push_back(kv);
        }
        return ki;
    }
    
    std::vector<rosplan_knowledge_msgs::KnowledgeItem> MongoInterface::getEntries(std::string ns, mongo::Query query) {
        db_cursor cursor = client.query(ns, query);
        std::vector<rosplan_knowledge_msgs::KnowledgeItem> ret;
        while(cursor->more()) {
            ret.push_back(bsonToKnowledgeItem(cursor->next()));
        }
        return ret;
    }
    
    void MongoInterface::addEntry(std::string ns, mongo::BSONObj b) {
        client.insert(ns, b);
        client.ensureIndex(ns, BSON("knowledge_type" << 1 << "instance_type" << 1 << "instance_name" << 1 << "attribute_name" << 1));
    }

    bool MongoInterface::isEntry(std::string ns, mongo::BSONObj b) {
        db_cursor cursor = client.query(ns, b);
        return cursor->more();
    }
    
    void MongoInterface::rmEntry(std::string ns, mongo::BSONObj b) {
        client.remove(ns, b);
    }
    
    // Knowledge specific functions
    
    // Instances
    std::vector<rosplan_knowledge_msgs::KnowledgeItem> MongoInterface::getInstances(std::string instance_type) {
        mongo::BSONObjBuilder b;
        b.append("knowledge_type", rosplan_knowledge_msgs::KnowledgeItem::INSTANCE);
        if(instance_type.compare("")!=0) b.append("instance_type", instance_type);
        return getKnowledge(b.obj());
    }
    
    void MongoInterface::removeInstances(std::string instance_type) {
        mongo::BSONObjBuilder b;
        b.append("knowledge_type", rosplan_knowledge_msgs::KnowledgeItem::INSTANCE);
        if(instance_type.compare("")!=0) b.append("instance_type", instance_type);
        return rmKnowledge(b.obj());
    }

    // Facts
    std::vector<rosplan_knowledge_msgs::KnowledgeItem> MongoInterface::getFacts(std::string attribute_name) {
        mongo::BSONObjBuilder b;
        b.append("knowledge_type", rosplan_knowledge_msgs::KnowledgeItem::FACT);
        if(attribute_name.compare("")!=0) b.append("attribute_name", attribute_name);
        return getKnowledge(b.obj());
    }
    
    void MongoInterface::removeFacts(std::string attribute_name) {
        mongo::BSONObjBuilder b;
        b.append("knowledge_type", rosplan_knowledge_msgs::KnowledgeItem::FACT);
        if(attribute_name.compare("")!=0) b.append("attribute_name", attribute_name);
        return rmKnowledge(b.obj());
    }

    // Functions
    std::vector<rosplan_knowledge_msgs::KnowledgeItem> MongoInterface::getFunctions(std::string attribute_name) {
        mongo::BSONObjBuilder b;
        b.append("knowledge_type", rosplan_knowledge_msgs::KnowledgeItem::FUNCTION);
        if(attribute_name.compare("")!=0) b.append("attribute_name", attribute_name);
        return getKnowledge(b.obj());
    }
    
    void MongoInterface::removeFunctions(std::string attribute_name) {
        mongo::BSONObjBuilder b;
        b.append("knowledge_type", rosplan_knowledge_msgs::KnowledgeItem::FUNCTION);
        if(attribute_name.compare("")!=0) b.append("attribute_name", attribute_name);
        return rmKnowledge(b.obj());
    }
    
    // Goals
    std::vector<rosplan_knowledge_msgs::KnowledgeItem> MongoInterface::getGoals(std::string attribute_name) {
        mongo::BSONObjBuilder b;
        b.append("knowledge_type", rosplan_knowledge_msgs::KnowledgeItem::FACT);
        if(attribute_name.compare("")!=0) b.append("attribute_name", attribute_name);
        return getEntries(ns_g, b.obj());
    }
    
    void MongoInterface::removeGoals(std::string attribute_name) {
        mongo::BSONObjBuilder b;
        b.append("knowledge_type", rosplan_knowledge_msgs::KnowledgeItem::FACT);
        if(attribute_name.compare("")!=0) b.append("attribute_name", attribute_name);
        return rmEntry(ns_g, b.obj());
    }
}
