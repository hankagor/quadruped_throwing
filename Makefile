HRI_COMMAND = rosrun go1_software hri
THROW_COMMAND = rosrun go1_software throw 1.8
OVERHEAD_COMMAND = rosrun go1_software overhead
SPAWN1_COMMAND = rosrun gazebo_ros spawn_model -sdf -file /home/hanka/catkin_ws_go1_full/src/go1_full/unitree_ros/unitree_gazebo/worlds/building_editor_models/throwing_objects/smaller.sdf -model smaller_box
SPAWN2_COMMAND = rosrun gazebo_ros spawn_model -sdf -file /home/hanka/catkin_ws_go1_full/src/go1_full/unitree_ros/unitree_gazebo/worlds/building_editor_models/throwing_objects/box.sdf -model smaller_box
SPAWN_COMMAND = rostopic pub -1 /spawnobj_cmd std_msgs/Float64 "data: 0.5"
DELETE1_COMMAND = rosservice call /gazebo/delete_model "model_name: 'smaller_box'"
DELETE_COMMAND = rosservice call /gazebo/delete_model "model_name: 'my_box'"
PUBLISH3_COMMAND = rostopic pub -1 /catchobj_params std_msgs/Float64MultiArray "data: [0.1, -0.1, 0, 0.3, 1]"
PUBLISH4_COMMAND = rostopic pub -1 /catchobj_params std_msgs/Float64MultiArray "data: [0.1, -0.3, 0, 0.4, 1]"
PUBLISH5_COMMAND = rostopic pub -1 /catchobj_params std_msgs/Float64MultiArray "data: [0.0, -0.5, 0, 0.4, 1]"
PUBLISH1_COMMAND = rostopic pub -1 /catchobj_params std_msgs/Float64MultiArray "data: [0.1, 0.15, 0.0, -0.05, 1]"
PUBLISH2_COMMAND = rostopic pub -1 /catchobj_params std_msgs/Float64MultiArray "data: [0.1, 0.3, 0.0, 0.2, 1]"
PUBLISH_COMMAND = rostopic pub -1 /catchobj_params std_msgs/Float64MultiArray "data: [0.0, -0.35, 0.0, 0.15, 2]"
GRIP_COMMAND = rostopic pub -1 /catchobj_params std_msgs/Float64MultiArray "data: [0.1, -0.6, 0, 0.4, 3]"
LIST_MODELS_COMMAND = rosservice call /gazebo/get_world_properties
ROBOT_NAME = go1_old
DELAY = 5
BOXPUB_COMMAND = rosrun go1_software boxpub

SHELL := /bin/bash

START_COMMAND = rostopic pub -1 /catchobj_params std_msgs/Float64MultiArray "data: [0.1, -0.6, 0, 0.5, 0.8]"
GRIP1_COMMAND = rostopic pub -1 /catchobj_params std_msgs/Float64MultiArray "data: [0.1, -0.6, 0, 0.5, 3]"

PLATFORM = rosrun gazebo_ros spawn_model -sdf -file /home/hanka/catkin_ws_go1_full/src/go1_full/unitree_ros/unitree_gazebo/worlds/building_editor_models/throwing_objects/platform.sdf -model platform
DELETE_PLATFORM = rosservice call /gazebo/delete_model "model_name: 'platform'"
# Default target to run all commands in sequence
	
all: run_hri throw
	
# Target to run the HRI command from go1_software package
run_hri:
	@echo "Deleting all objects except the robot..."
	@$(LIST_MODELS_COMMAND) | grep model_names | sed 's/\[//g' | sed 's/\]//g' | sed 's/,//g' | while read model; do \
		if [ "$$model" != "$(ROBOT_NAME)" ]; then \
			rosservice call /gazebo/delete_model "{model_name: '$$model'}"; \
		fi \
	done
	@echo "Moving to start position"
	$(HRI_COMMAND)

# Target to publish to /catchobj_params
publish:
	@echo "Preparing for throwing"
	$(PUBLISH_COMMAND)

# Target to spawn the smaller.sdf object in Gazebo
spawn:
	@echo "Spawning object..."
	$(SPAWN1_COMMAND)
	
grip:
	

# Target to run the throw command from go1_software package
throw_cmd:
	@echo "Throwing..."
	$(THROW_COMMAND)
	@sleep ${DELAY}

# Target to delete the smaller_box object in Gazebo
delete:
	@echo "Deleting object..."
	$(DELETE1_COMMAND)

# Run all commands in sequence: HRI, publish, spawn, throw, delete
throw: 
	$(PUBLISH5_COMMAND)
	$(SPAWN1_COMMAND)
	$(PUBLISH_COMMAND)
	$(THROW_COMMAND)
	$(DELETE1_COMMAND)
	@echo "Finished all commands."
	
.PHONY: loop_commands

loop_commands:
	for i in {6..25}; do \
	    goal=$$(echo "0.5 + 0.1 * $$i" | bc); \
	    for j in {1..6}; do \
	        $(PUBLISH5_COMMAND); \
	        $(SPAWN1_COMMAND); \
	        $(PUBLISH_COMMAND); \
	        rosrun go1_software throw $$goal; \
	        echo "Executed THROW_COMMAND with goal=$$goal (Iteration $$j)"; \
	        $(DELETE1_COMMAND); \
	    done; \
	done; \
	echo "Finished all commands."



overhead:
	$(OVERHEAD_COMMAND)
over_head: 
	$(PUBLISH1_COMMAND)
	$(PUBLISH2_COMMAND)
	
pickup: 
	$(PLATFORM)
	$(START_COMMAND)
	$(SPAWN2_COMMAND)
	$(GRIP1_COMMAND)
	$(DELETE_PLATFORM)
	$(GRIP_COMMAND)
	$(THROW_COMMAND)
	$(DELETE1_COMMAND)
	
	
platform:
	$(PLATFORM)
	
throw_hw:
	overhead
	publish
	@sleep ${DELAY}
	$(GRIP1_COMMAND)
	throw_cmd

track:
	$(BOXPUB_COMMAND)
	
# Clean target (optional)
clean:
	@echo "Nothing to clean."


.PHONY: all run_hri publish spawn throw_cmd delete clean throw

