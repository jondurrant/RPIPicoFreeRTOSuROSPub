/*
 * PubEntities.cpp
 *
 * Test agent to publish to two topics
 * 	pico_rnd: Random number on Topic
 * 	joint_states: Rotational join in Radians
 *  Created on: 5 Jul 2023
 *      Author: jondurrant
 */

#include "PubEntities.h"
#include "pico/rand.h"
#include "uRosBridge.h"

extern"C"{
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/joint_state.h>
#include "pico_usb_transports.h"
#include "rosidl_runtime_c/string_functions.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"
}


PubEntities::PubEntities() {
	// TODO Auto-generated constructor stub

}

PubEntities::~PubEntities() {
	// TODO Auto-generated destructor stub
}

/***
 * Get the static depth required in words
 * @return - words
 */
configSTACK_DEPTH_TYPE PubEntities::getMaxStackSize(){
	return 256;
}


/***
 * Run loop for the agent.
 */
void PubEntities::run(){
	std_msgs__msg__Int32 msg;

	sensor_msgs__msg__JointState joint_state_msg;
	struct timespec ts;
	clockid_t id;
	double a= 0.0;
	double speed = 0.01;
	bool acc = true;
	const char *jointName = "post_to_table";

	//Setup the joint state msg
	sensor_msgs__msg__JointState__init(&joint_state_msg);
	rosidl_runtime_c__double__Sequence__init(&joint_state_msg.position, 1);
	joint_state_msg.position.data[0] = a;
	joint_state_msg.position.size = 1;
	joint_state_msg.position.capacity = 1;
	rosidl_runtime_c__String__Sequence__init(&joint_state_msg.name, 1);
	if (!rosidl_runtime_c__String__assign(&joint_state_msg.name.data[0], jointName)){
		printf("ERROR: Joined assignment failed\n");
	}
	joint_state_msg.name.size=1;
	joint_state_msg.name.capacity=1;


	for (;;){
		if (xCount != 0){
			//Populate the Int32 message for pico_rnd
			msg.data = get_rand_32() % 0x7FFFFFFF;
			if (!uRosBridge::getInstance()->publish(
					&xPublisher,
					&msg,
					this,
					NULL)){
				printf("RND Pub failed\n");
			}

			// Calc radian for Rotating and accelerating turn table
			a += speed;
			if (a > 3.14){
				a = - 3.14;
				if (acc){
					speed = speed * 1.2;
				} else {
					speed = speed * 0.8;
				}

				if (speed > 0.5){
					//speed = 0.01;
					acc = false;
				}
				if (speed < 0.01){
					//speed = 0.01;
					acc = true;
				}
			}

			//Populate the Joint possition message
		    clock_gettime(id, &ts);
		    joint_state_msg.header.stamp.sec = ts.tv_sec;
		    joint_state_msg.header.stamp.nanosec = ts.tv_nsec;
		    joint_state_msg.position.data[0] = a;
		    if (!uRosBridge::getInstance()->publish(&xPubJoint,
		    		&joint_state_msg,
					this,
					NULL)){
				printf("Joint Pub failed\n");
			}

		}
		vTaskDelay(10);
	}
}

/***
 * Provide a count of the number of entities
 * @return number of entities >=0
 */
uint PubEntities::getCount(){
	return xCount;
}

/***
 * Create the publishing entities
 * @param node
 * @param support
 */
void PubEntities::createEntities(rcl_node_t *node, rclc_support_t *support){
	rclc_publisher_init_default(
			&xPublisher,
			node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
			"pico_rnd");

	rclc_publisher_init_default(
			&xPubJoint,
			node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
			"joint_states");
	xCount = 2;
}

/***
 * Destroy the publishing entities
 * @param node
 * @param support
 */
void PubEntities::destroyEntities(rcl_node_t *node, rclc_support_t *support){
	xCount = 0;
	rcl_publisher_fini(&xPublisher, node);
	rcl_publisher_fini(&xPubJoint, node);
}



