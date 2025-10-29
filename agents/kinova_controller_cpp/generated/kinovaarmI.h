/*
 *    Copyright (C) 2025 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef KINOVAARM_H
#define KINOVAARM_H

// Ice includes
#include <Ice/Ice.h>
#include <KinovaArm.h>

#include "../src/specificworker.h"


class KinovaArmI : public virtual RoboCompKinovaArm::KinovaArm
{
public:
	KinovaArmI(GenericWorker *_worker, const size_t id);
	~KinovaArmI();

	bool closeGripper(const Ice::Current&);
	RoboCompKinovaArm::TPose getCenterOfTool(RoboCompKinovaArm::ArmJoints referencedTo, const Ice::Current&);
	RoboCompKinovaArm::TGripper getGripperState(const Ice::Current&);
	RoboCompKinovaArm::TJoints getJointsState(const Ice::Current&);
	RoboCompKinovaArm::TToolInfo getToolInfo(const Ice::Current&);
	void moveJointsWithAngle(RoboCompKinovaArm::TJointAngles angles, const Ice::Current&);
	void moveJointsWithSpeed(RoboCompKinovaArm::TJointSpeeds speeds, const Ice::Current&);
	void openGripper(const Ice::Current&);
	void setCenterOfTool(RoboCompKinovaArm::TPose pose, RoboCompKinovaArm::ArmJoints referencedTo, const Ice::Current&);
	bool setGripperPos(float pos, const Ice::Current&);

private:

	GenericWorker *worker;
	size_t id;

	// Array handlers for each method
	std::array<std::function<bool(void)>, 1> closeGripperHandlers;
	std::array<std::function<RoboCompKinovaArm::TPose(RoboCompKinovaArm::ArmJoints)>, 1> getCenterOfToolHandlers;
	std::array<std::function<RoboCompKinovaArm::TGripper(void)>, 1> getGripperStateHandlers;
	std::array<std::function<RoboCompKinovaArm::TJoints(void)>, 1> getJointsStateHandlers;
	std::array<std::function<RoboCompKinovaArm::TToolInfo(void)>, 1> getToolInfoHandlers;
	std::array<std::function<void(RoboCompKinovaArm::TJointAngles)>, 1> moveJointsWithAngleHandlers;
	std::array<std::function<void(RoboCompKinovaArm::TJointSpeeds)>, 1> moveJointsWithSpeedHandlers;
	std::array<std::function<void(void)>, 1> openGripperHandlers;
	std::array<std::function<void(RoboCompKinovaArm::TPose, RoboCompKinovaArm::ArmJoints)>, 1> setCenterOfToolHandlers;
	std::array<std::function<bool(float)>, 1> setGripperPosHandlers;

};

#endif
