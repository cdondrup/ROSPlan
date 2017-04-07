# ROSPlan Framework

The main ROSPlan website and documentation is available here:
http://kcl-planning.github.io/ROSPlan/

The ROSPlan framework has been extended by Christian Dondrup to be able to interact with Petri-Net Plans and to make the knowledgebase persistent.

## Installation

For installation and usage instructions, please visit: http://protolab.aldebaran.com:9000/mummer/documentations/wikis/wp4-demo-system

## Running ROSPlan

### Persistent version

This require mongodb to be running:

```
roslaunch mongodb_store mongodb_store.launch
```

Start the planning framework:

```
roslaunch rosplan_planning_system planning_system_knowledge.launch domain_path:=/path/to/my/domain/file.pddl persistent:=true
```

This will create a DB for the KB and keep it in mongodb.

### Non-persistent version

```
roslaunch rosplan_planning_system planning_system_knowledge.launch domain_path:=/path/to/my/domain/file.pddl persistent:=false
```

This will only prserve the knowledge while rosplan is running. Once it is restarted the KB needs to be reinitialised. This does not require mongodb to be running.
