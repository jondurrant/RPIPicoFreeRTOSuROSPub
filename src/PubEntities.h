/*
 * PubEntities.h
 *
 * Test agent to publish to two topics
 * 	pico_rnd: Random number on Topic
 * 	joint_states: Rotational join in Radians
 *
 *
 *  Created on: 5 Jul 2023
 *      Author: jondurrant
 */

#ifndef FREERTOSRECON_SRC_PUBENTITIES_H_
#define FREERTOSRECON_SRC_PUBENTITIES_H_

#include "Agent.h"
#include "uRosEntities.h"

class PubEntities : public Agent, public uRosEntities {
public:
	PubEntities();
	virtual ~PubEntities();

	/***
	 * Create the publishing entities
	 * @param node
	 * @param support
	 */
	virtual void createEntities(rcl_node_t *node, rclc_support_t *support);

	/***
	 * Destroy the publishing entities
	 * @param node
	 * @param support
	 */
	virtual void destroyEntities(rcl_node_t *node, rclc_support_t *support);

	/***
	 * Provide a count of the number of entities
	 * @return number of entities >=0
	 */
	virtual uint getCount();

protected:

	/***
	 * Run loop for the agent.
	 */
	virtual void run();


	/***
	 * Get the static depth required in words
	 * @return - words
	 */
	virtual configSTACK_DEPTH_TYPE getMaxStackSize();

private:
	rcl_publisher_t xPublisher;
	rcl_publisher_t xPubJoint;
	uint xCount = 0;
};

#endif /* FREERTOSRECON_SRC_PUBENTITIES_H_ */
