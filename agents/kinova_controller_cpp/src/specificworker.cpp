/*
 *    Copyright (C) 2024 by YOUR NAME HERE
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
#include "specificworker.h"

#include <ranges>
#include <rapplication/rapplication.h>
#include <cppitertools/sliding_window.hpp>
#include "api_kinova_controller.h"

SpecificWorker::SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startup_check) : GenericWorker(configLoader, tprx)
{
this->startup_check_flag = startup_check;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		#ifdef HIBERNATION_ENABLED
			hibernationChecker.start(500);
		#endif

		
		// Example statemachine:
		/***
		//Your definition for the statesmachine (if you dont want use a execute function, use nullptr)
		states["CustomState"] = std::make_unique<GRAFCETStep>("CustomState", period, 
															std::bind(&SpecificWorker::customLoop, this),  // Cyclic function
															std::bind(&SpecificWorker::customEnter, this), // On-enter function
															std::bind(&SpecificWorker::customExit, this)); // On-exit function

		//Add your definition of transitions (addTransition(originOfSignal, signal, dstState))
		states["CustomState"]->addTransition(states["CustomState"].get(), SIGNAL(entered()), states["OtherState"].get());
		states["Compute"]->addTransition(this, SIGNAL(customSignal()), states["CustomState"].get()); //Define your signal in the .h file under the "Signals" section.

		//Add your custom state
		statemachine.addState(states["CustomState"].get());
		***/

		statemachine.setChildMode(QState::ExclusiveStates);
		statemachine.start();

		auto error = statemachine.errorString();
		if (error.length() > 0){
			qWarning() << error;
			throw error;
		}
		
	}
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
	//G->write_to_json_file("./"+agent_name+".json");

	delete api_controller;
}

void SpecificWorker::initialize()
{
	std::cout << "Initialize worker" << std::endl;

	// 2D widget
//dsr update signals
	//connect(G.get(), &DSR::DSRGraph::update_node_signal, this, &SpecificWorker::modify_node_slot);
	//connect(G.get(), &DSR::DSRGraph::update_edge_signal, this, &SpecificWorker::modify_edge_slot);
	//connect(G.get(), &DSR::DSRGraph::update_node_attr_signal, this, &SpecificWorker::modify_node_attrs_slot);
	//connect(G.get(), &DSR::DSRGraph::update_edge_attr_signal, this, &SpecificWorker::modify_edge_attrs_slot);
	//connect(G.get(), &DSR::DSRGraph::del_edge_signal, this, &SpecificWorker::del_edge_slot);
	//connect(G.get(), &DSR::DSRGraph::del_node_signal, this, &SpecificWorker::del_node_slot);
	graph_viewer = std::make_unique<DSR::DSRViewer>(this, G, current_opts, main);

	rt = G->get_rt_api();

	this->resize(1200, 1200);

	new_speeds = std::vector<float>(7, 0.0);

	auto ip = configLoader.get<std::string>("ip");

	std::cout << "Trying to create the api_controller" << std::endl;
	api_controller = new api_kinova_controller(ip);
	std::cout << "api_controller created" << std::endl;

	// DISCOMMENT IF YOU WANT TO SET THE ARM IN THE HOME POSITION AT THE START OF THE CONTROLLER
	// api_controller->move_to_selected_pose("Home");

	joints = api_controller->get_joints_info();
	gripper = api_controller->get_gripper_state();
	tool_state = api_controller->get_tool_state();
	// api_controller->print_joints_info();

	// update_dsr_joints();
	show_forward_kinematics();

	std::cout << "\033[2J";

	// TEST TO THE CONTACTILE SENSOR DISCOMMENT ONLY IN CASE YOU WANT TO TEST IT
	// test_contactile();
}

void SpecificWorker::compute()
{
	// TEST TO THE GRIPPER SPEED MOVEMENT DISCOMMENT ONLY IN CASE YOU WANT TO TEST IT
	// gripper_test_loop();

	// THE SPEED MOVE TEST DISCOMMENT ONLY IN CASE YOU WANT TO TEST IT
	// test_speed_move();

	joints = api_controller->get_joints_info();
	gripper = api_controller->get_gripper_state();
	tool_state = api_controller->get_tool_state();

	if (speed_check_flag)
	{
		const auto now = std::chrono::system_clock::now();
		const auto ms_now = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
		if (abs(ms_now - last_time_speed_check) > 1000)
		{
			new_speeds = std::vector<float>(7, 0.0);
			speed_check_flag = false;
		}
		api_controller->move_joints_with_speeds(new_speeds);
	}

	// show_tool_state();

	// update_dsr_joints();
	// show_forward_kinematics();

}


void SpecificWorker::gripper_test_loop()
{
	static bool close = true;
	RoboCompKinovaArm::TGripper gripper_state = api_controller->get_gripper_state();
	std::cout << "Gripper state: " << gripper_state.distance << std::endl;
	if (close)
	{
		api_controller->move_gripper_with_vel(0.005);
		if (gripper_state.distance <= 0.05)
			close = false;
	}
	else
	{
		api_controller->move_gripper_with_vel(-0.005);
		if (gripper_state.distance >= 0.95)close = true;
	}
}

void SpecificWorker::test_contactile()
{
	std::cout << "Test contactile" << std::endl;
	const bool close = KinovaArm_closeGripper();
	std::cout << "Gripper close: " << close << std::endl;
}

void SpecificWorker::test_speed_move()
{
	std::cout << "Test speed_move" << std::endl;
	RoboCompKinovaArm::TJointSpeeds kinova_speeds;
	kinova_speeds.jointSpeeds = std::vector<float> (7, 0.0);
	kinova_speeds.jointSpeeds[6] = 5.0;
	for (auto speed : kinova_speeds.jointSpeeds)
	{
		std::cout << "Speed: " << speed << std::endl;
	}
	KinovaArm_moveJointsWithSpeed(kinova_speeds);
}

std::vector<float> SpecificWorker::get_joints_angles()
{
	std::vector<float> angles(joints.joints.size());
	for (auto joint:joints.joints)
		angles.push_back(joint.angle);

	return angles;
}

void SpecificWorker::update_dsr_joints() {
	int joint_id = 0;
	for (const auto &names: json_links_names | iter::sliding_window(2))
	{
		if (auto parent = G->get_node(names[0]); parent.has_value())
		{
			if (auto child = G ->get_node(names[1]); child.has_value())
			{
				if (auto edge = rt->get_edge_RT(parent.value(), child.value().id()); edge.has_value())
				{
					auto rot = G->get_attrib_by_name<rt_rotation_euler_xyz_att>(edge.value());
					auto trans = G->get_attrib_by_name<rt_translation_att>(edge.value());
					if (rot.has_value() && trans.has_value())
					{
						auto rot_val = rot.value().get();
						auto trans_val = trans.value().get();
						rot_val[2] = qDegreesToRadians(joints.joints[joint_id].angle);
						rt->insert_or_assign_edge_RT(parent.value(), child.value().id(), trans_val, rot_val);
						joint_id++;
					}
				}
			}
		}
	}
}

void SpecificWorker::show_forward_kinematics()
{
	std::cout << "Show forward kinematics" << std::endl;
	const auto pose = api_controller->get_forward_kinematics();
	const auto translation = std::get<0>(pose);
	const auto rotation = std::get<1>(pose);
	const auto quat_rot = euler_to_quaternion(rotation);
	std::cout << "Translation: " << std::endl;
	std::cout <<"x: " << translation[0] << "| y: " << translation[1] << "| z: " << translation[2] << std::endl;
	std::cout << "Euler rotation: " << std::endl;
	std::cout <<"theta_x: " << rotation[0] << "| theta_y: " << rotation[1] << "| theta_z: " << rotation[2] << std::endl;
	std::cout <<"Rotation expresed as Quaternion: " << std::endl;
	std::cout <<"w: " << quat_rot.w << "| x:" << quat_rot.x << "| y: " << quat_rot.y << "| z: " << quat_rot.z << std::endl;
}

SpecificWorker::Quaternion SpecificWorker::euler_to_quaternion(const std::vector<float> &euler_thetas)
{
	// Convertir ángulos de Euler de grados a radianes
	auto theta_x = euler_thetas[0] * M_PI / 180.0;
	auto theta_y = euler_thetas[1] * M_PI / 180.0;
	auto theta_z = euler_thetas[2] * M_PI / 180.0;

	// Calcular los senos y cosenos de la mitad de los ángulos
	double cy = cos(theta_y * 0.5);
	double sy = sin(theta_y * 0.5);
	double cp = cos(theta_x * 0.5);
	double sp = sin(theta_x * 0.5);
	double cr = cos(theta_z * 0.5);
	double sr = sin(theta_z * 0.5);

	// Calcular el cuaternión
	Quaternion q;
	q.w = cr * cp * cy + sr * sp * sy;
	q.x = sr * cp * cy - cr * sp * sy;
	q.y = cr * sp * cy + sr * cp * sy;
	q.z = cr * cp * sy - sr * sp * cy;

	return q;
}

// void SpecificWorker::show_tool_state() {
// 	    // Configuración de formato
//     std::cout << fixed << std::setprecision(4);
//
//     // Línea separadora
//     const string separator = "----------------------------------------------------";
//
//     // Encabezado
//     std::cout << "\n" << separator << "\n";
//     std::cout << "               ESTADO DEL BRAZO KINOVA\n";
//     std::cout << separator << "\n";
//
//     // Sección 1: Pose (Posición y Orientación)
//     std::cout << "POSE:\n";
//     std::cout << "  Posición (m):   X = " << setw(8) << tool_state.pose.x
//          << "   Y = " << setw(8) << tool_state.pose.y
//          << "   Z = " << setw(8) << tool_state.pose.z << "\n";
//     std::cout << "  Orientación (θ): X = " << setw(8) << tool_state.pose_theta.x
//          << "   Y = " << setw(8) << tool_state.pose_theta.y
//          << "   Z = " << setw(8) << tool_state.pose_theta.z << "\n";
//
//     // Sección 2: Twist (Velocidad Lineal y Angular)
//     std::cout << separator << "\n";
//     std::cout << "TWIST:\n";
//     std::cout << "  Vel. Lineal (m/s):  X = " << setw(8) << tool_state.twist_linear.x
//          << "   Y = " << setw(8) << tool_state.twist_linear.y
//          << "   Z = " << setw(8) << tool_state.twist_linear.z << "\n";
//     std::cout << "  Vel. Angular (rad/s): X = " << setw(8) << tool_state.twist_angular.x
//          << "   Y = " << setw(8) << tool_state.twist_angular.y
//          << "   Z = " << setw(8) << tool_state.twist_angular.z << "\n";
//
//     // Sección 3: Wrench (Fuerza y Torque Externo)
//     std::cout << separator << "\n";
//     std::cout << "WRENCH EXTERNO:\n";
//
//     // Fuerza - con color rojo si es significativa
//     std::cout << "  Fuerza (N):     X = ";
//     if(fabs(tool_state.external_wrench_force.x) > 1.0) std::cout << "\033[31m"; // Rojo para valores > 1N
//     std::cout << setw(8) << tool_state.external_wrench_force.x << "\033[0m";
//
//     std::cout << "   Y = ";
//     if(fabs(tool_state.external_wrench_force.y) > 1.0) std::cout << "\033[31m";
//     std::cout << setw(8) << tool_state.external_wrench_force.y << "\033[0m";
//
//     std::cout << "   Z = ";
//     if(fabs(tool_state.external_wrench_force.z) > 1.0) std::cout << "\033[31m";
//     std::cout << setw(8) << tool_state.external_wrench_force.z << "\033[0m\n";
//
//     // Torque - con color amarillo si es significativo
//     std::cout << "  Torque (Nm):    X = ";
//     if(fabs(tool_state.external_wrench_torque.x) > 0.5) std::cout << "\033[33m"; // Amarillo para > 0.5Nm
//     std::cout << setw(8) << tool_state.external_wrench_torque.x << "\033[0m";
//
//     std::cout << "   Y = ";
//     if(fabs(tool_state.external_wrench_torque.y) > 0.5) std::cout << "\033[33m";
//     std::cout << setw(8) << tool_state.external_wrench_torque.y << "\033[0m";
//
//     std::cout << "   Z = ";
//     if(fabs(tool_state.external_wrench_torque.z) > 0.5) std::cout << "\033[33m";
//     std::cout << setw(8) << tool_state.external_wrench_torque.z << "\033[0m\n";
//
//     std::cout << separator << "\n\n";
// }

void SpecificWorker::moveCursor(int row, int col) {
	std::cout << "\033[" << row << ";" << col << "H";
}

void SpecificWorker::clearScreen() {
	std::cout << "\033[J";
}

void SpecificWorker::show_tool_state() {
    // Mover cursor a la posición inicial (1,1)
    moveCursor(1, 1);

    // Configuración de formato
    std::cout << fixed << setprecision(3);

    // Encabezado estático (solo se imprime una vez)
    static bool first_call = true;
    if(first_call) {
        std::cout << "----------------------------------------------------\n";
        std::cout << "               ESTADO DEL BRAZO KINOVA (TIEMPO REAL) \n";
        std::cout << "----------------------------------------------------\n";
        std::cout << "POSE:\n";
        std::cout << "  Posición (m):   X =       Y =       Z =      \n";
        std::cout << "  Orientación (θ): X =       Y =       Z =      \n";
        std::cout << "----------------------------------------------------\n";
        std::cout << "TWIST:\n";
        std::cout << "  Vel. Lineal (m/s):  X =       Y =       Z =      \n";
        std::cout << "  Vel. Angular (rad/s): X =       Y =       Z =      \n";
        std::cout << "----------------------------------------------------\n";
        std::cout << "WRENCH EXTERNO:\n";
        std::cout << "  Fuerza (N):     X =       Y =       Z =      \n";
        std::cout << "  Torque (Nm):    X =       Y =       Z =      \n";
        std::cout << "----------------------------------------------------\n";
        first_call = false;
    }

    // Actualizar solo los valores (sobreescribiendo)

    // Posición
    moveCursor(5, 22); std::cout << setw(8) << tool_state.pose.x;
    moveCursor(5, 31); std::cout << setw(8) << tool_state.pose.y;
    moveCursor(5, 40); std::cout << setw(8) << tool_state.pose.z;

    // Orientación
    moveCursor(6, 22); std::cout << setw(8) << tool_state.poseTheta.x;
    moveCursor(6, 31); std::cout << setw(8) << tool_state.poseTheta.y;
    moveCursor(6, 40); std::cout << setw(8) << tool_state.poseTheta.z;

    // Velocidad lineal
    moveCursor(9, 22); std::cout << setw(8) << tool_state.twistLinear.x;
    moveCursor(9, 31); std::cout << setw(8) << tool_state.twistLinear.y;
    moveCursor(9, 40); std::cout << setw(8) << tool_state.twistLinear.z;

    // Velocidad angular
    moveCursor(10, 22); std::cout << setw(8) << tool_state.twistAngular.x;
    moveCursor(10, 31); std::cout << setw(8) << tool_state.twistAngular.y;
    moveCursor(10, 40); std::cout << setw(8) << tool_state.twistAngular.z;

    // Fuerza externa (con color)
    moveCursor(13, 22);
    if(fabs(tool_state.externalWrenchForce.x) > 1.0) std::cout << "\033[31m";
    std::cout << setw(6) << tool_state.externalWrenchForce.x << "\033[0m";

    moveCursor(13, 31);
    if(fabs(tool_state.externalWrenchForce.y) > 1.0) std::cout << "\033[31m";
    std::cout << setw(6) << tool_state.externalWrenchForce.y << "\033[0m";

    moveCursor(13, 40);
    if(fabs(tool_state.externalWrenchForce.z) > 1.0) std::cout << "\033[31m";
    std::cout << setw(6) << tool_state.externalWrenchForce.z << "\033[0m";

    // Torque externo (con color)
    moveCursor(14, 22);
    if(fabs(tool_state.externalWrenchTorque.x) > 0.5) std::cout << "\033[33m";
    std::cout << setw(6) << tool_state.externalWrenchTorque.x << "\033[0m";

    moveCursor(14, 31);
    if(fabs(tool_state.externalWrenchTorque.y) > 0.5) std::cout << "\033[33m";
    std::cout << setw(6) << tool_state.externalWrenchTorque.y << "\033[0m";

    moveCursor(14, 40);
    if(fabs(tool_state.externalWrenchTorque.z) > 0.5) std::cout << "\033[33m";
    std::cout << setw(6) << tool_state.externalWrenchTorque.z << "\033[0m";

    // Mover cursor al final para evitar interferencias
    moveCursor(16, 1);
    std::cout.flush();
}

///////////////////////////////////////////////////////////////////////////////////////

bool SpecificWorker::KinovaArm_closeGripper()
{
	float force = 0;
	float gripper_dist = gripper.distance;
	while (force < 10.0 && gripper_dist < 0.9)
	{
		api_controller->move_gripper_with_vel(-0.005);
		auto tactileValues = contactile_proxy->getValues();
		force = fabs(tactileValues.left.x) + fabs(tactileValues.left.y) + fabs(tactileValues.left.z)
		+ fabs(tactileValues.right.x) + fabs(tactileValues.right.y) + fabs(tactileValues.right.z);
		gripper = api_controller->get_gripper_state();
		gripper_dist = gripper.distance;
	}

	api_controller->move_gripper_with_vel(0.0);

	if (force > 2.0)
		return true;

	return false;
}

RoboCompKinovaArm::TPose SpecificWorker::KinovaArm_getCenterOfTool(RoboCompKinovaArm::ArmJoints referencedTo)
{
	RoboCompKinovaArm::TPose ret{};
	//implementCODE

	return ret;
}

RoboCompKinovaArm::TGripper SpecificWorker::KinovaArm_getGripperState()
{
	return gripper;
}

RoboCompKinovaArm::TJoints SpecificWorker::KinovaArm_getJointsState()
{
	return joints;
}

RoboCompKinovaArm::TToolInfo SpecificWorker::KinovaArm_getToolInfo()
{

	return tool_state;
}

void SpecificWorker::KinovaArm_moveJointsWithAngle(RoboCompKinovaArm::TJointAngles angles)
{
	std::vector<float> degrees(angles.jointAngles.size());
    std::transform(angles.jointAngles.begin(), angles.jointAngles.end(), degrees.begin(), 
                  [](float rad) { return qRadiansToDegrees(rad); });

	api_controller->move_joints_with_angles(degrees);
}

void SpecificWorker::KinovaArm_moveJointsWithSpeed(RoboCompKinovaArm::TJointSpeeds speeds)
{

    std::transform(speeds.jointSpeeds.begin(), speeds.jointSpeeds.end(), new_speeds.begin(), 
                  [](float rad) { return qRadiansToDegrees(rad); });
	speed_check_flag = true;
	const auto now = std::chrono::system_clock::now();
	last_time_speed_check = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
}

void SpecificWorker::KinovaArm_openGripper()
{
	std::cout << "Opening the gripper" << std::endl;
	api_controller->move_gripper_with_pos(0.0);
}

void SpecificWorker::KinovaArm_setCenterOfTool(RoboCompKinovaArm::TPose pose, RoboCompKinovaArm::ArmJoints referencedTo)
{
	//implementCODE
}

bool SpecificWorker::KinovaArm_setGripperPos(float pos)
{

	return api_controller->move_gripper_with_pos(std::clamp(pos, 0.0f, 1.0f));
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::emergency()
{
	std::cout << "Emergency worker" << std::endl;
	//emergencyCODE
	//
	//if (SUCCESSFUL) //The componet is safe for continue
	//  emmit goToRestore()
}

//Execute one when exiting to emergencyState
void SpecificWorker::restore()
{
	std::cout << "Restore worker" << std::endl;
	//restoreCODE
	//Restore emergency component
}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, QCoreApplication::instance(), SLOT(quit()));
	return 0;
}


/**************************************/
// From the RoboCompContactile you can call this methods:
// RoboCompContactile::FingerTips this->contactile_proxy->getValues()

/**************************************/
// From the RoboCompContactile you can use this types:
// RoboCompContactile::FingerTip
// RoboCompContactile::FingerTips

/**************************************/
// From the RoboCompKinovaArm you can use this types:
// RoboCompKinovaArm::TPose
// RoboCompKinovaArm::TAxis
// RoboCompKinovaArm::TToolInfo
// RoboCompKinovaArm::TGripper
// RoboCompKinovaArm::TJoint
// RoboCompKinovaArm::TJoints
// RoboCompKinovaArm::TJointSpeeds
// RoboCompKinovaArm::TJointAngles

