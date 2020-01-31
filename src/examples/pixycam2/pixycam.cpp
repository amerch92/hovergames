/****************************************************************************
 *
 *   Copyright 2019 NXP
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *	  contributors may be used to endorse or promote products derived
 *	  from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
* @file pixycam.cpp
*
* Example for reading deteced blocks and print them on the console
*
* @author Leo Mustafa
*/

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include "Pixy2I2C_PX4.h"

// inlcude all of the references for getting the current gps position
#include "uORB/topics/vehicle_gps_position.h"
#include <uORB/uORB.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/parameter_update.h>

extern "C" __EXPORT int pixycam_main(int argc, char *argv[]);

// create global_pos_sub that grabs the vehicle_global_position
// I'm going to use it to transmit the location back
int global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));

int pixycam_main(int argc, char *argv[])
{
	// this is just for initialization
	PX4_INFO("Hello pixycam driver");
	usleep(5000);

	Pixy2 pixy;


	if (pixy.init() == 0) {

		pixy.getVersion();
		pixy.version->print();
		usleep(1000);

		while (1) {

			int i;
			// grab blocks!
			pixy.ccc.getBlocks();

			// If there are detect blocks, print them!
			if (pixy.ccc.numBlocks) {
				// I don't care about how many blocks there are but if any are detected
				// then I want to print the location of those components
				bool pos_updated;
				
				// update the position using orb_check
				orb_check(global_pos_sub, &pos_updated);
				
				// print that we detected something at this position
				// this will allow the drone to just have a set route, print off all
				// of the positions and then we can individually review each one of these
				// positions
				printf("Detected at location: @u", global_pos_sub);

				}
			}
			
			usleep(500);
		}
	}

	PX4_INFO("Exit pixycam driver");

	return 0;
}
