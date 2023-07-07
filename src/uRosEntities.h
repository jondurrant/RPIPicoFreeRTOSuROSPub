/*
 * uRosEntities.h
 *
 * Abstract class to connect the uRosBridge to Entities as tasks
 *
 *  Created on: 5 Jul 2023
 *      Author: jondurrant
 */

#ifndef FREERTOSRECON_SRC_UROSENTITIES_H_
#define FREERTOSRECON_SRC_UROSENTITIES_H_

#include "pico/stdlib.h"
#include "pico/types.h"

extern"C"{
#include <rcl/rcl.h>
#include <rclc/rclc.h>
}


/**
 * Status of the Publish
 */
enum uRosPubStatus {
	PubOK,    /**< PubOK */
	PubFailed,/**< PubFailed */
	PubCleared/**< PubCleared */
} ;

class uRosEntities {
public:
	uRosEntities();
	virtual ~uRosEntities();

	/***
	 * Create the entities (Publishers)
	 * @param node
	 * @param support
	 */
	virtual void createEntities(rcl_node_t *node, rclc_support_t *support)=0;

	/***
	 * Destroy the entities
	 * @param node
	 * @param support
	 */
	virtual void destroyEntities(rcl_node_t *node, rclc_support_t *support)=0;

	/***
	 * Return the number of entities
	 * @return
	 */
	virtual uint getCount()=0;

	/***
	 * Call back on a publish to show msg has been completed.
	 * Can be used to free up allocated memory
	 * @param msg - ROS Msg
	 * @param args - Arguments passed into the publish step
	 * @param status -
	 */
	virtual void pubComplete(void *msg, void *args, uRosPubStatus status);
};

#endif /* FREERTOSRECON_SRC_UROSENTITIES_H_ */
