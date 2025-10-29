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
#include "kinovaarmI.h"

KinovaArmI::KinovaArmI(GenericWorker *_worker, const size_t id): worker(_worker), id(id)
{
	closeGripperHandlers = {
		[this]() { return worker->KinovaArm_closeGripper(); }
	};

	getCenterOfToolHandlers = {
		[this](auto a) { return worker->KinovaArm_getCenterOfTool(a); }
	};

	getGripperStateHandlers = {
		[this]() { return worker->KinovaArm_getGripperState(); }
	};

	getJointsStateHandlers = {
		[this]() { return worker->KinovaArm_getJointsState(); }
	};

	getToolInfoHandlers = {
		[this]() { return worker->KinovaArm_getToolInfo(); }
	};

	moveJointsWithAngleHandlers = {
		[this](auto a) { return worker->KinovaArm_moveJointsWithAngle(a); }
	};

	moveJointsWithSpeedHandlers = {
		[this](auto a) { return worker->KinovaArm_moveJointsWithSpeed(a); }
	};

	openGripperHandlers = {
		[this]() { return worker->KinovaArm_openGripper(); }
	};

	setCenterOfToolHandlers = {
		[this](auto a, auto b) { return worker->KinovaArm_setCenterOfTool(a, b); }
	};

	setGripperPosHandlers = {
		[this](auto a) { return worker->KinovaArm_setGripperPos(a); }
	};

}


KinovaArmI::~KinovaArmI()
{
}


bool KinovaArmI::closeGripper(const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < closeGripperHandlers.size())
		return  closeGripperHandlers[id]();
	else
		throw std::out_of_range("Invalid closeGripper id: " + std::to_string(id));

}

RoboCompKinovaArm::TPose KinovaArmI::getCenterOfTool(RoboCompKinovaArm::ArmJoints referencedTo, const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < getCenterOfToolHandlers.size())
		return  getCenterOfToolHandlers[id](referencedTo);
	else
		throw std::out_of_range("Invalid getCenterOfTool id: " + std::to_string(id));

}

RoboCompKinovaArm::TGripper KinovaArmI::getGripperState(const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < getGripperStateHandlers.size())
		return  getGripperStateHandlers[id]();
	else
		throw std::out_of_range("Invalid getGripperState id: " + std::to_string(id));

}

RoboCompKinovaArm::TJoints KinovaArmI::getJointsState(const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < getJointsStateHandlers.size())
		return  getJointsStateHandlers[id]();
	else
		throw std::out_of_range("Invalid getJointsState id: " + std::to_string(id));

}

RoboCompKinovaArm::TToolInfo KinovaArmI::getToolInfo(const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < getToolInfoHandlers.size())
		return  getToolInfoHandlers[id]();
	else
		throw std::out_of_range("Invalid getToolInfo id: " + std::to_string(id));

}

void KinovaArmI::moveJointsWithAngle(RoboCompKinovaArm::TJointAngles angles, const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < moveJointsWithAngleHandlers.size())
		 moveJointsWithAngleHandlers[id](angles);
	else
		throw std::out_of_range("Invalid moveJointsWithAngle id: " + std::to_string(id));

}

void KinovaArmI::moveJointsWithSpeed(RoboCompKinovaArm::TJointSpeeds speeds, const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < moveJointsWithSpeedHandlers.size())
		 moveJointsWithSpeedHandlers[id](speeds);
	else
		throw std::out_of_range("Invalid moveJointsWithSpeed id: " + std::to_string(id));

}

void KinovaArmI::openGripper(const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < openGripperHandlers.size())
		 openGripperHandlers[id]();
	else
		throw std::out_of_range("Invalid openGripper id: " + std::to_string(id));

}

void KinovaArmI::setCenterOfTool(RoboCompKinovaArm::TPose pose, RoboCompKinovaArm::ArmJoints referencedTo, const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < setCenterOfToolHandlers.size())
		 setCenterOfToolHandlers[id](pose, referencedTo);
	else
		throw std::out_of_range("Invalid setCenterOfTool id: " + std::to_string(id));

}

bool KinovaArmI::setGripperPos(float pos, const Ice::Current&)
{

    #ifdef HIBERNATION_ENABLED
		worker->hibernationTick();
	#endif
    
	if (id < setGripperPosHandlers.size())
		return  setGripperPosHandlers[id](pos);
	else
		throw std::out_of_range("Invalid setGripperPos id: " + std::to_string(id));

}

