/*
 * uRosEntities.cpp
 *
 * `Abstrac
 *
 *  Created on: 5 Jul 2023
 *      Author: jondurrant
 */

#include "uRosEntities.h"

uRosEntities::uRosEntities() {
	// TODO Auto-generated constructor stub

}

uRosEntities::~uRosEntities() {
	// TODO Auto-generated destructor stub
}

/***
 * Call back on a publish to show msg has been completed.
 * Can be used to free up allocated memory
 * @param msg - ROS Msg
 * @param args - Arguments passed into the publish step
 * @param status -
 */
void uRosEntities::pubComplete(
		void *msg,
		void *args,
		uRosPubStatus status){
	//NOP
}
