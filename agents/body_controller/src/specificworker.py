#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2025 by YOUR NAME HERE
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
#
from itertools import count
import cProfile
import pstats
import io
from functools import wraps
import threading

# def profile_qt(sort_by='cumulative', limit=10):
#     """Decorador que muestra resultados de profiling ordenados y formateados
    
#     Args:
#         sort_by (str): Criterio de ordenación ('time', 'cumulative', 'calls', etc.)
#         limit (int): Número máximo de líneas a mostrar
#     """
#     def decorator(func):
#         @wraps(func)
#         def wrapper(*args, **kwargs):
#             profiler = cProfile.Profile()
#             profiler.enable()
            
#             result = func(*args, **kwargs)
            
#             profiler.disable()
            
#             # Configurar la visualización de estadísticas
#             stats = pstats.Stats(profiler)
            
#             print("\n" + "="*80)
#             print(f"PROFILING RESULTS for {func.__name__}")
#             print("="*80)
            
#             # Ordenar y mostrar las estadísticas
#             stats.strip_dirs().sort_stats(sort_by).print_stats(limit)
            
#             # Mostrar también las llamadas que más tiempo consumen individualmente
#             print("\nTop time consumers per call:")
#             stats.sort_stats('time').print_stats(5)
            
#             return result
#         return wrapper
#     return decorator


from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QApplication
from rich.console import Console
from rich.text import Text
from genericworker import *
import interfaces as ifaces
from time import time, sleep

sys.path.append('/opt/robocomp/lib')
sys.path.append('src')
console = Console(highlight=False)

from pydsr import *
import math

import matplotlib
matplotlib.use('Agg')

import swift
import roboticstoolbox as rtb
import spatialmath as sm
import numpy as np
import qpsolvers as qp
import spatialgeometry as sg
from roboticstoolbox import Robot
from multiprocessing import Process, Queue

from kinova_gen3 import KinovaGen3



try:
    import setproctitle
    setproctitle.setproctitle(os.path.basename(os.getcwd()))
except:
    pass


SCALE = 0.001
ROBOT_DSR = ("P3bot", 200)

# Pick and Place Task States
class TaskState:
    IDLE = "IDLE"
    OFFSET_ALIGNMENT = "OFFSET_ALIGNMENT"
    MOVING_TO_TARGET = "MOVING_TO_TARGET"
    CLOSING_GRIPPER = "CLOSING_GRIPPER"
    PICKING_UP = "PICKING_UP"
    MOVING_SIDEWAYS = "MOVING_SIDEWAYS"
    PLACING_DOWN = "PLACING_DOWN"
    OPENING_GRIPPER = "OPENING_GRIPPER"
    MOVING_BACKWARD = "MOVING_BACKWARD"
    DONE = "DONE"


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, configData, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map, configData)
        self.Period = self.configData["Period"]["Compute"]
        self.useRTPose = self.configData["useRTPose"]
        self.automatic = self.configData["automatic"]
        self.simulated = self.configData["simulated"]
        self.base = self.configData["base"]
        self.directKinematic = self.configData["directKinematic"]

        assert isinstance(self.simulated, bool), f"Simulated must be bool, dont {type(self.simulated)}"
        assert isinstance(self.directKinematic, bool), f"directKinematic must be bool, dont {type(self.directKinematic)}"

        self.pose = None
        self.loop_count = 0

        #region Objects and poses
        self.COLLISION_BODY = [[["low_body", "right_arm_forearm_link"], ["left_arm", "left_arm_tool_frame"]], 
                          [ ["low_body", "right_arm_tool_frame"], ["left_arm", "left_arm_forearm_link"]]]
        
        self.home =  np.radians(np.array([[40,-120,60,-130,-20,-65, 85], [-40,-120,-60,-130,20,-65, 85]], dtype=np.float64))
        self.pick =  np.radians(np.array([[90,-120,80,-130,-20, 45, 95], [-90,-120,-80,-130, 20, 45, 85]], dtype=np.float64))
        self.gain = np.array([1, 1, 1, 1.6, 1.6, 1.6])
        #endregion

        #region DSR
        self.rt = rt_api(self.g)
        try:
            signals.connect(self.g, signals.UPDATE_EDGE, self.update_edge)
            signals.connect(self.g, signals.DELETE_EDGE, self.delete_edge)
            console.print("signals connected")
        except RuntimeError as e:
            print(e)
        #endregion

        # region Kinova Gen 3 robot initialization
        self.kinova_arms = [None, None]
        if not self.simulated:
            self.kinova_arms = [self.kinovaarm_proxy, self.kinovaarm1_proxy]
        #endregion

        toolOffset = sm.SE3.Tz(0.135) * sm.SE3.Rz(np.deg2rad(180))
        T = sm.SE3(0, 0, 0.04)

        self.bodyOffset = sm.SE3.Rz(0)
        self.targetOffset = sm.SE3.Rz(np.deg2rad(180)) * sm.SE3.Ry(np.deg2rad(180)) 


        if startup_check:
            self.startup_check()
        else:

            self.env = swift.Swift()
            self.env.launch(realtime=True)
            self.env.set_camera_pose([-2, 3, 0.7], [-2, 0.0, 0.5])

            #region P3Bot
            # self.p3bot = Robot.URDF("/home/robolab/software/robotics-toolbox-python/rtb-data/rtbdata/xacro/p3bot_description/urdf/P3Bot_scaled.urdf")
            self.p3bot = rtb.models.P3Bot()
            if self.base:
                self.p3bot.qdlim = [ 1.5, 0.4] + [2]*14
            else:
                self.p3bot.qdlim = [0]*2 + [2.5]*14
            

            self.bodyOffset = sm.SE3.Rz(0)
            self.targetOffset = sm.SE3.Rz(np.deg2rad(180)) * sm.SE3.Ry(np.deg2rad(180)) 

            Rz = sm.SE3.Rz(3.14)
            self.p3bot.base = T * Rz
            self.env.add(self.p3bot)
            #endregion

            #region getGoal
            self.goal_axes = [sg.Axes(0.1)]*2
            self.deadManButton = [False]*2
            self.gripperOpening = [0]*2
            self.target = [None]*2
            self.poseController = [np.array((0,0,0,0,0,0))]*2
            self.haptics = np.array([0]*2)
            
            
            # Offset target parameters
            self.offset_target = [False]*2  # Track if currently in offset phase
            self.original_target = [None]*2  # Store original target during offset phase
            self.offset_distance = 0.3  # Distance to offset from target (meters)
            
            # Pick and Place Task Management
            self.task_state = [TaskState.IDLE]*2  # Current task state for each arm
            self.task_pick_height = [None]*2  # Height at which the object was picked
            self.place_position = [None]*2  # Position where to place the object
            self.place_offset = 1.5  # Distance to move sideways (meters)
            self.gripper_closing_time = [0]*2  # Timer for gripper closing
            self.gripper_timeout = 1.0  # Time to wait for gripper to close (seconds)
            #endregion

            print(self.p3bot.grippers[0].tool)

            #region Tool Points
            self.collisions = [sg.Cuboid((0.5, 1, 0.07), pose=sm.SE3(-1, 0, 0.7), color=(0, 1, 0,0.25))]
            for i in range(len(self.collisions)):
                self.env.add(self.collisions[i])

            self.collisions_tool = []
            for i in range(len(self.p3bot.grippers)):
                self.p3bot.grippers[i].tool *= toolOffset
                frame = sg.Axes(0.1, pose=self.p3bot.grippers[i].tool)
                frame.attach_to(self.p3bot.grippers[i].links[0])
                self.env.add(frame)

                self.collisions_tool.append([sg.Cuboid((0.065, 0.08, 0.045), pose=self.p3bot.grippers[i].tool, color=(1, 0, 0,0.25)),
                                             sg.Cuboid((0.03, 0.098, 0.15), pose=self.p3bot.grippers[i].tool, color=(1, 0, 0,0.25))])
                self.env.add(self.collisions_tool[i][0])
                self.env.add(self.collisions_tool[i][1])
            #endregion

            self.set_all_joints(self.pick)

            for i in range(len(self.p3bot.grippers)): 
                self.update_collisions_tool(self.p3bot, i)
                self.env.add(self.goal_axes[i]) 

            self.targetNode = None

            if self.automatic:
                table_node = self.g.get_node("Table1")
                if table_node is None:
                    print("Table node not found")
                    return
                target_edge = Edge(table_node.id, ROBOT_DSR[1], "TARGET", self.agent_id)
                self.g.insert_or_assign_edge(target_edge)
            else:
                targets = self.g.get_edges_by_type("TARGET")
                print("Targets encontrados:", targets)
                for t in targets:
                    print("Target edge:", t.origin)
                    if t.destination == ROBOT_DSR[1]:
                        print(f"Target encontrado: {t.attrs["rt_translation"].value}, {t.attrs["rt_rotation_euler_xyz"].value}")
                        pose = t.attrs["rt_translation"].value
                        rot = t.attrs["rt_rotation_euler_xyz"].value
                        self.change_target(rot=rot, translate=pose)
                        self.targetNode = t.origin
                        break
            #endregion

            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)
            # print("Valores articulares actuales:", self.p3bot.link_dict)


    def __del__(self):
        """Destructor"""

    def set_velocity_joints(self, arm: int, velocity: list[float]) -> None:
        """Set the velocity of the robot arm joints.

        Args:
            arm (int): Index of the robot arm (0-based).
            velocity (list[float]): List of velocities (in rad/s) for each joint.

        Raises:
            AssertionError: If the arm index is out of bounds.
            Exception: If setting velocity fails (logged to console).
        """
        assert self.simulated or arm < len(self.kinova_arms), f"Robot has {len(self.kinova_arms)} arms, tried to access arm {arm + 1}"
        assert 7 == len(velocity), f"Robot has {7} joins, tried use {len(velocity)}" 
        try:
            # console.print(Text(f"Set velocity {velocity}", "green"))

            if not self.simulated:
                speed = ifaces.RoboCompKinovaArm.TJointSpeeds(jointSpeeds=ifaces.RoboCompKinovaArm.Speeds(velocity))
                self.kinova_arms[arm].moveJointsWithSpeed(speed)
            self.p3bot.qd[2 + arm * 7 : 9 + arm * 7] = velocity
        
        except Exception as e:
            console.print(Text(f"Failed to set joint velocities: {e}", "red"))
            console.print_exception()
        # finally:
        #     self.p3bot.q[2 + arm * 7 : 9 + arm * 7] = self.get_joints(arm)

    def set_all_joints(self, poses: list[list[float]]) -> None:
        """Mueve todos los brazos robóticos a los ángulos articulares especificados en paralelo.
        
        Args:
            poses (list[list[float]]): Ángulos objetivo para cada brazo (en radianes).
        
        Raises:
            AssertionError: Si el número de poses no coincide con el número de brazos.
        """
        assert len(poses) == len(self.kinova_arms), \
            f"El robot tiene {len(self.kinova_arms)} brazos, pero se proporcionaron {len(poses)} poses"

        threads = []
        for i in range(len(self.kinova_arms)):
            # Crear un hilo por brazo
            thread = threading.Thread(
                target=self.set_joints,
                args=(i, poses[i])  # Argumentos: (arm, pose)
            )
            thread.start()
            threads.append(thread)  # Guardar referencia

        # Esperar a que todos los hilos terminen
        for thread in threads:
            thread.join()
                
    def set_joints(self, arm: int, pose: list[float]) -> None:
        """Move the robot arm to the specified joint angles.

        Args:
            arm (int): Index of the robot arm (0-based).
            pose (list[float]): Target joint angles (in radians).

        Raises:
            AssertionError: If the arm index is out of bounds.
            Exception: If setting joint angles fails (logged to console).
        """
        assert arm < len(self.kinova_arms), f"Robot has {len(self.kinova_arms)} arms, tried to access arm {arm + 1}"
        assert 7 == len(pose), f"Robot has {7} joins, tried use {len(pose)}" 
        try:
            counter = 0
            while not np.allclose(self.p3bot.q[2 + arm * 7 : 9 + arm * 7], pose, atol=0.01):
                if counter % 1000 == 0:
                    console.print(Text(f"Set pose {pose}", "green"))
                    if self.simulated:
                        self.p3bot.q[2 + arm * 7 : 9 + arm * 7] = pose
                    else:
                        self.set_velocity_joints(arm, [0]*7)
                        angles = ifaces.RoboCompKinovaArm.TJointAngles(jointAngles=ifaces.RoboCompKinovaArm.Angles(np.array(pose)))
                        self.kinova_arms[arm].moveJointsWithAngle(angles)
                
                sleep(0.005)
                self.p3bot.q[2 + arm * 7 : 9 + arm * 7] = self.get_joints(arm)
                # self.p3bot.q[2 + arm * 7 : 9 + arm * 7] = pose
                # print(self.p3bot.q[2 + arm * 7 : 9 + arm * 7], pose, "\n\n\n")
                self.env.step(0)
                counter+=1
        except Exception as e:
            console.print(Text(f"Failed to set joint angles: {e}", "red"))
            console.print_exception()
        finally:
            self.p3bot.q[2 + arm * 7 : 9 + arm * 7] = self.get_joints(arm)
            self.env.step(0)




    def get_joints(self, arm: int) -> list[float]:
        """Retrieve the current joint angles of the robot arm.

        Args:
            arm (int): Index of the robot arm (0-based).

        Returns:
            list[float]: Current joint angles (in radians).

        Raises:
            AssertionError: If the arm index is out of bounds.
            Exception: If fetching joint angles fails (logged to console).
        """
        assert self.simulated or arm < len(self.kinova_arms), f"Robot has {len(self.kinova_arms)} arms, tried to access arm {arm + 1}"
        try:
            if self.simulated:
                return self.p3bot.q[2 + arm * 7 : 9 + arm * 7].tolist()

            else:
                data = self.kinova_arms[arm].getJointsState()
                angles = np.array([joint.angle for joint in data.joints])
                angles[angles > np.pi] -= 2*np.pi  # Normalize angles >180° to [-180°, 180°]
                return angles.tolist()
        except Exception as e:
            console.print(Text(f"Failed to get joint angles: {e}", "red"))
            console.print_exception()
            return []



    def noisePose(self):
        current_base = self.p3bot.base  # Transformación actual (sm.SE3)
        # Ruido gaussiano en X, Y, Z (media=0, desviación estándar=0.01 metros)
        translation_noise = np.random.normal(0, 0.001, size=3)  # [Δx, Δy, Δz]
        # Crear una transformación de traslación con el ruido
        T_noise = sm.SE3(translation_noise)
        # Ruido en la rotación (pequeño ángulo en radianes, ej: σ=0.1 rad)
        rotation_noise_z = np.random.normal(0, 0.01)  # Ruido en rotación Z
        # Crear una transformación de rotación con el ruido
        R_noise = sm.SE3.Rz(rotation_noise_z)
        self.p3bot.base = current_base * R_noise * T_noise

    def update_collisions(self, pose:sm.SE3.Trans):
        for i in range(len(self.collisions)):
            self.collisions[i].T = pose * self.cubes_positions[i]

    @QtCore.Slot()
    # @profile_qt(sort_by='cumulative', limit=15)
    def compute(self):
        t1= time()
        #Update pose in swift
        if self.pose is not None:
            T = sm.SE3(self.pose[0:3])
            RPY = sm.SE3.RPY(self.pose[3:6])
            self.pose = None
            self.p3bot.base = T *self.bodyOffset * RPY
            # print(f"New pose: {T}, {RPY}")

        for arm in range(len(self.kinova_arms)):self.p3bot.q[2 + arm * 7 : 9 + arm * 7] = self.get_joints(arm)
        # self.update_collisions(self.p3bot.base)

        #Go to target
        if self.targetNode is not None:
            armSelect = 0#(self.loop_count//2) % 2
            distance = np.linalg.norm(self.p3bot.base.t - self.target[armSelect].t)
            for arm in range(len(self.kinova_arms)):
                self.update_collisions_tool(self.p3bot, arm)

            if self.directKinematic:                
                arrived, qd = self.direct_kinematic_robot(self.p3bot, armSelect, self.target[armSelect].A)
            else:
                arrived, qd = self.step_robot(self.p3bot, armSelect, self.target[armSelect].A, self.collisions_tool[armSelect]+self.collisions)

            #Block arm to far targets
            if distance > 2.5:
                qd[2:] = [0]*(len(qd)-2)


            # print(f"\rDistance: {distance:0.2f}, velocity:", end="")
            #for vel in qd: print(f" {vel:0.2f}", end="") 
            # print(f"\radv:{qd[1]*1000:.2f} | rot:{qd[0]:.2f} ejes {qd[2:]}", end="")

            #Move motors
            if qd is not None:
                if self.simulated:
                    self.p3bot.qd[:2] = qd[:2]
                else:
                    try:
                        self.omnirobot_proxy.setSpeedBase(0, qd[1]*1000, qd[0])
                        pass
                    except Ice.ConnectionRefusedException:
                        console.print_exception()
                self.set_velocity_joints(armSelect, qd[2 : 9])

            self.env.step(0.05)

            base_new = self.p3bot.fkine(self.p3bot._q, end=self.p3bot.links[2])
            self.p3bot._T = base_new.A
            self.p3bot.q[:2] = 0
            
            if arrived:
                self.handle_task_state(armSelect)
                
        #print(time()-t1)
        return True
    
    def handle_task_state(self, arm: int) -> None:
        """
        Handle state transitions in the pick and place task
        
        Args:
            arm (int): Arm index
        """
        current_state = self.task_state[arm]
        print(f"[Task] arm {arm} state: {current_state}")
        
        if current_state == TaskState.OFFSET_ALIGNMENT:
            # Reached offset position, now move to original target
            print(f"arm {arm}: Offset alignment complete. Moving to original target...")
            self.task_state[arm] = TaskState.MOVING_TO_TARGET
            self.target[arm] = self.original_target[arm]
            self.offset_target[arm] = False
            
        elif current_state == TaskState.MOVING_TO_TARGET:
            # Reached original target, start closing gripper
            print(f"arm {arm}: Reached target position. Closing gripper...")
            self.task_state[arm] = TaskState.CLOSING_GRIPPER
            # self.task_state[arm] = TaskState.PICKING_UP            #For simplicity, we directly transition to picking up after reaching target
            self.gripper_closing_time[arm] = time()
            self.pick_up_object(arm)
            
        elif current_state == TaskState.CLOSING_GRIPPER:
            # Gripper closing in progress, wait for timeout
            elapsed = time() - self.gripper_closing_time[arm]
            if elapsed >= self.gripper_timeout:
                print(f"arm {arm}: Gripper closed. Starting to pick up...")
                self.task_state[arm] = TaskState.PICKING_UP
                # Move upward 20cm from CURRENT position, keep same XY and rotation
                current_pose = self.p3bot.fkine(self.p3bot.q, end=self.p3bot.grippers[arm])
                lift_height = current_pose.t[2] + 0.2  # Move up 20cm from current Z
                # Create new pose with same XY and rotation, only Z increases
                lift_translation = np.array([current_pose.t[0], current_pose.t[1], lift_height])
                # lift_pose = sm.SE3(lift_translation) * sm.SO3(current_pose.A[0:3, 0:3])
                temp_lift_pose = self.compute_lift_position(self.target[arm], offset_dist=0.2)
                self.target[arm] = temp_lift_pose
            
        elif current_state == TaskState.PICKING_UP:
            # Object lifted, now move sideways
            print(f"arm {arm}: Lifted object. Moving sideways...")
            # Get current pose after lifting
            current_lifted_pose = self.p3bot.fkine(self.p3bot.q, end=self.p3bot.grippers[arm])
            # Compute and set sideways target (moving right 30cm)
            sideways_target = self.compute_sideways_position(current_lifted_pose, offset_dist=0.3)
            self.target[arm] = sideways_target
            self.task_state[arm] = TaskState.MOVING_SIDEWAYS
            
        elif current_state == TaskState.MOVING_SIDEWAYS:
            # Reached sideways position, now move down for placing
            print(f"arm {arm}: At sideways position. Moving down to place...")
            # Get current pose after moving sideways
            current_sideways_pose = self.p3bot.fkine(self.p3bot.q, end=self.p3bot.grippers[arm])
            # Compute and set place down target (moving down 20cm)
            place_down_target = self.compute_place_down_position(current_sideways_pose, offset_dist=0.2)
            self.target[arm] = place_down_target
            self.task_state[arm] = TaskState.PLACING_DOWN
            
        elif current_state == TaskState.PLACING_DOWN:
            # At place height, open gripper
            print(f"arm {arm}: At place location. Opening gripper...")
            self.task_state[arm] = TaskState.OPENING_GRIPPER
            self.open_gripper(arm)
            self.gripper_closing_time[arm] = time()
            
        elif current_state == TaskState.OPENING_GRIPPER:
            # Wait for gripper to open
            elapsed = time() - self.gripper_closing_time[arm]
            if elapsed >= self.gripper_timeout:
                print(f"arm {arm}: Gripper opened. Moving backward...")
                # Get current pose after opening gripper
                current_pose = self.p3bot.fkine(self.p3bot.q, end=self.p3bot.grippers[arm])
                # Compute and set backward target (moving back 20cm in -Z direction)
                backward_target = self.compute_backward_position(current_pose, offset_dist=0.2)
                self.target[arm] = backward_target
                self.task_state[arm] = TaskState.MOVING_BACKWARD
        
        elif current_state == TaskState.MOVING_BACKWARD:
            # Reached backward position, task complete
            print(f"arm {arm}: Pick and place complete!")
            self.set_all_joints(self.pick)      # Move back to pick position after completing the task
            self.task_state[arm] = TaskState.DONE
            
            try:
                self.omnirobot_proxy.setSpeedBase(0, 0, 0)
            except Ice.ConnectionRefusedException:
                console.print_exception()
            
            print(f"DELETE EDGE: TARGET")
            self.g.delete_edge(self.targetNode, ROBOT_DSR[1], "TARGET")
            self.targetNode = None
            self.set_velocity_joints(arm, [0]*7)
            
            if self.automatic:
                self.loop_count += 2
                table_node = self.g.get_node(f"Table{(self.loop_count % 4) +1}")
                if table_node is None:
                    print("Root node not found")
                    return
                target_edge = Edge(table_node.id, ROBOT_DSR[1], "TARGET", self.agent_id)
                self.g.insert_or_assign_edge(target_edge)
        
        elif current_state == TaskState.DONE:
            # Task complete, stay idle
            pass

    def update_collisions(self, pose:sm.SE3.Trans):
        for i in range(len(self.collisions)):
            self.collisions[i].T = pose * self.cubes_positions[i]

    def update_collisions_tool(self, r:rtb.ERobot, gripperSelect:int):
        gripper = r.grippers[gripperSelect]   # por ejemplo
        wTe = r.fkine(r.q, end=gripper)
        self.collisions_tool[gripperSelect][0].T = wTe * sm.SE3([-0.04, 0, -0.15])
        self.collisions_tool[gripperSelect][1].T = wTe * sm.SE3([0, 0, -0.05])

    def change_target(self, translate:np.ndarray, rot:np.ndarray):
        print(f"Changed goal {translate}, {rot}")
        armSelect =  0#(self.loop_count//2) % 2
        # self.set_joints(armSelect, self.pick[armSelect])

        # Change the target position of the end-effector
        # T = sm.SE3(translate*SCALE)
        T = sm.SE3(translate)
        RPY = sm.SE3.RPY(rot)

        self.target[armSelect] = T * self.targetOffset * RPY
        self.goal_axes[armSelect].T = self.target[armSelect]
        
        # Initialize Pick and Place Task
        self.task_state[armSelect] = TaskState.OFFSET_ALIGNMENT
        self.offset_target[armSelect] = True
        self.original_target[armSelect] = self.target[armSelect].copy()
        
        # Compute and set offset position
        offset_pose = self.compute_offset_position(self.original_target[armSelect])
        self.target[armSelect] = offset_pose
        print(f"Task started: OFFSET_ALIGNMENT phase")

    def compute_offset_position(self, target_pose: sm.SE3, offset_dist: float = None) -> sm.SE3:
        """
        Computes an offset position for aligning the gripper before approaching the target.
        The offset is applied along the negative Z-axis (approaching direction).
        
        Args:
            target_pose: The original target pose (as sm.SE3)
            offset_dist: Distance to offset (default: self.offset_distance)
            
        Returns:
            A new sm.SE3 pose offset backward along the approach direction
        """
        if offset_dist is None:
            offset_dist = self.offset_distance
            
        # Create offset along negative Z-axis (gripper approaches along Z)
        offset = sm.SE3.Tz(-offset_dist)
        
        # Apply offset in the target frame
        offset_pose = target_pose * offset
        
        return offset_pose

    def compute_lift_position(self, target_pose: sm.SE3, offset_dist: float = None) -> sm.SE3:
        """
        Compute lift position: moves UP 20cm from target using -X axis.
        The offset is applied along the negative X-axis (lifting direction).
        Axis alignment: -X is upward, -Y is right, -Z is backward
        
        Args:
            target_pose: The original target pose (as sm.SE3)
            offset_dist: Distance to offset (default: 0.2 for 20cm)
            
        Returns:
            A new sm.SE3 pose lifted upward
        """
        if offset_dist is None:
            offset_dist = 0.2  # 20cm default lift
            
        # Create offset along negative X-axis (upward)
        offset = sm.SE3.Tx(-offset_dist)
        
        # Apply offset in the target frame
        lifted_pose = target_pose * offset
        
        return lifted_pose
    
    def compute_sideways_position(self, current_pose: sm.SE3, offset_dist: float = 0.3) -> sm.SE3:
        """
        Compute sideways position: moves RIGHT 30cm from current lifted position using -Y axis.
        Axis alignment: -X is upward, -Y is right, -Z is backward
        
        Args:
            current_pose: The current lifted pose (as sm.SE3)
            offset_dist: Distance to offset (default: 0.3 for 30cm)
            
        Returns:
            A new sm.SE3 pose moved to the right
        """
        # Create offset along negative Y-axis (right)
        offset = sm.SE3.Ty(-offset_dist)
        
        # Apply offset in the current frame
        sideways_pose = current_pose * offset
        
        return sideways_pose
    
    def compute_place_down_position(self, current_pose: sm.SE3, offset_dist: float = 0.2) -> sm.SE3:
        """
        Compute place down position: moves DOWN 20cm from current sideways position using +X axis.
        Axis alignment: -X is upward, -Y is right, -Z is backward
        
        Args:
            current_pose: The current sideways pose (as sm.SE3)
            offset_dist: Distance to offset (default: 0.2 for 20cm)
            
        Returns:
            A new sm.SE3 pose moved downward to table height
        """
        # Create offset along positive X-axis (downward, opposite of lifting)
        offset = sm.SE3.Tx(offset_dist)
        
        # Apply offset in the current frame
        down_pose = current_pose * offset
        
        return down_pose
    
    def compute_backward_position(self, current_pose: sm.SE3, offset_dist: float = 0.2) -> sm.SE3:
        """
        Compute backward movement: moves BACKWARD 20cm from current position using -Z axis.
        Axis alignment: -X is upward, -Y is right, -Z is backward
        
        Args:
            current_pose: The current pose (as sm.SE3)
            offset_dist: Distance to offset (default: 0.2 for 20cm)
            
        Returns:
            A new sm.SE3 pose moved backward
        """
        # Create offset along negative Z-axis (backward)
        offset = sm.SE3.Tz(-offset_dist)
        
        # Apply offset in the current frame
        backward_pose = current_pose * offset
        
        return backward_pose
    
    def set_gripper_position(self, arm: int, position: float) -> None:
        """
        Set gripper position. 0 = open, 1 = closed
        
        Args:
            arm (int): Arm index (0 or 1)
            position (float): Target position (0.0 to 1.0)
        """
        assert 0 <= position <= 1.0, f"Gripper position must be between 0 and 1, got {position}"
        try:
            if not self.simulated:
                self.kinova_arms[arm].setGripperPos(position)
                print(f"Gripper {arm} set to position {position}")
            else:
                print(f"[SIM] Gripper {arm} set to position {position}")
        except Exception as e:
            console.print(Text(f"Failed to set gripper position: {e}", "red"))
    
    def pick_up_object(self, arm: int) -> None:
        """
        Pick up the object: Close gripper and move upward
        
        Args:
            arm (int): Arm index
        """
        # Store the original target height as pick height (where the object is)
        self.task_pick_height[arm] = self.original_target[arm].t[2]  # Z coordinate from original target
        
        # Close gripper
        self.set_gripper_position(arm, 1.0)  # 1.0 = closed
        
        print(f"Picking up object at height {self.task_pick_height[arm]:.3f}m")
    
    def place_down_object(self, arm: int, place_position: sm.SE3) -> None:
        """
        Place down the object: Move to position and open gripper
        
        Args:
            arm (int): Arm index
            place_position (sm.SE3): Position where to place the object
        """
        self.target[arm] = place_position
        self.place_position[arm] = place_position
        print(f"Moving to place position")
    
    def open_gripper(self, arm: int) -> None:
        """
        Open the gripper to release the object
        
        Args:
            arm (int): Arm index
        """
        self.set_gripper_position(arm, 0.0)  # 0.0 = open
        print(f"Gripper {arm} opened")
    
    def compute_place_target(self, arm: int) -> sm.SE3:
        """
        Compute the target position for placing the object.
        Moves sideways (along X-axis) and maintains the same Z height as pickup.
        
        Args:
            arm (int): Arm index
            
        Returns:
            sm.SE3: Target pose for placing
        """
        original = self.original_target[arm]
        # Move sideways (offset in X direction) and keep same Z
        place_pose = sm.SE3(self.place_offset, 0, 0) * original
        return place_pose

    def direct_kinematic_robot(self, r: rtb.ERobot, gripperSelect, Tep):
        gripper = r.grippers[gripperSelect]
        ets = r.ets(end=gripper)
        indices = ets.jindices

        v, arrived = rtb.p_servo(r.fkine(r.q, end=gripper), Tep, gain=self.gain, threshold=0.005)
        qd = np.clip(np.linalg.pinv(ets.jacobe(r.q)) @ v, -r.qdlim[indices], r.qdlim[indices])[2:9]
                                        
        return arrived, qd
        
    def step_robot(self, r: rtb.ERobot, gripperSelect, Tep, collisions):
        n = 9
        gripper = r.grippers[gripperSelect]
        ets = r.ets(end=gripper)
        wTe = r.fkine(r.q, end=gripper)

        eTep = np.linalg.inv(wTe) @ Tep

        # Spatial error
        et = np.sum(np.abs(eTep[:3, -1]))

        # print("Spatial error: ", et)

        # Gain term (lambda) for control minimisation
        Y = 0.01

        # Quadratic component of objective function
        Q = np.eye(n + 6)

        # Joint velocity component of Q
        Q[: n, : n] *= Y
        Q[:3, :3] *= 1.0 / et

        # Slack component of Q
        Q[n:, n:] = (1.0 / et) * np.eye(6)

        v, _ = rtb.p_servo(wTe, Tep, 1.5)

        v[3:] *= 1.3

        # The equality contraints
        Aeq = np.c_[ets.jacobe(r.q), np.eye(6)]#TODO tool
        beq = v.reshape((6,))

        # The inequality constraints for joint limit avoidance
        Ain = np.zeros((n + 6, n + 6))
        bin = np.zeros(n + 6)

        # The minimum angle (in radians) in which the joint is allowed to approach
        # to its limit
        ps = 0.1

        # The influence angle (in radians) in which the velocity damper
        # becomes active
        pi = 0.9

        # Form the joint limit velocity damper
        Ain[: n, : n], bin[: n] = r.joint_velocity_damper(ps, pi, n)

        rot_boost = 1
        vel_decay = 1
        num_collisions = 0

        #################COLISIONS##################
        if collisions is not None:
            for i, body in enumerate(self.COLLISION_BODY[gripperSelect]):
                for collision in collisions:
                    c_Ain, c_bin = self.p3bot.link_collision_damper(
                            collision,
                            self.p3bot.q,
                            di=0.1, # Distancia mínima más pequeña (ej: 0.1 metros)
                            ds=0.05, # Ganancia más alta (ej: 0.1)
                            xi=1, # Mayor peso en la optimización
                            start= self.p3bot.link_dict[body[0]], 
                            end= self.p3bot.link_dict[body[1]]
                        )

                    # If there are any parts of the robot within the influence distance
                    # to the collision in the scene
                    if c_Ain is not None and c_bin is not None:
                        # print(f"{i}, colision {c_Ain.shape}, {c_bin.shape}")
                        # print(c_bin)
                        # print(c_Ain)
                        c_Ain = c_Ain[:, :10]#TODO investigar porque es 0 


                        c_Ain = np.c_[c_Ain, np.zeros((c_Ain.shape[0], n + 6 - c_Ain.shape[1]))]
                        num_collisions += c_bin.shape[0]

                        # if len(c_Ain) > 1 : vel_decay +=len(c_bin)*2

                        # Stack the inequality constraints
                        Ain = np.r_[Ain, c_Ain]
                        bin = np.r_[bin, c_bin]

        ############################

        # Linear component of objective function: the manipulability Jacobian
        c = np.concatenate(
            (np.zeros(2), -r.jacobm(start=r.links[3], end=gripper).reshape((n - 2,)), np.zeros(6))
        )

        # Get base to face end-effector
        kε = 0.5
        bTe = r.fkine(r.q, end=gripper, include_base=False).A
        θε = math.atan2(bTe[1, -1], bTe[0, -1])
        ε = kε * θε
        c[0] = -ε

        # The lower and upper bounds on the joint velocity and slack variable
        start = gripperSelect*7+2
        lb = -np.r_[r.qdlim[:2],r.qdlim[start:start+7], 10 * np.ones(6)]
        ub = np.r_[r.qdlim[:2], r.qdlim[start:start+7], 10 * np.ones(6)]

        # Solve for the joint velocities dq
        qd = qp.solve_qp(Q, c, Ain, bin, Aeq, beq, lb=lb, ub=ub, solver="piqp")
        arrived = False

        if qd is not None:
            qd = qd.copy() 

            # ret_qd = r.qd.copy()
            # ret_qd[toolPoint.jindices] = qd[toolPoint.jindices].copy()
            # print("antes", qd)
            # qd[0] = qd[0] * rot_boost
            # qd[2:] = qd[2:] / vel_decay
            # print("despues", qd)

            # if et > 0.5:
            #     qd *= 0.5
            # else:
            #     qd *= et if et > 0.25 else 1

            if et < 0.02:
                arrived = True
        else:
            console.print(Text("Optimización fallida.", "yellow"))
            qd = np.zeros(n)
        return arrived, qd

    def startup_check(self):
        print(f"Testing RoboCompKinovaArm.TPose from ifaces.RoboCompKinovaArm")
        test = ifaces.RoboCompKinovaArm.TPose()
        print(f"Testing RoboCompKinovaArm.TAxis from ifaces.RoboCompKinovaArm")
        test = ifaces.RoboCompKinovaArm.TAxis()
        print(f"Testing RoboCompKinovaArm.TToolInfo from ifaces.RoboCompKinovaArm")
        test = ifaces.RoboCompKinovaArm.TToolInfo()
        print(f"Testing RoboCompKinovaArm.TGripper from ifaces.RoboCompKinovaArm")
        test = ifaces.RoboCompKinovaArm.TGripper()
        print(f"Testing RoboCompKinovaArm.TJoint from ifaces.RoboCompKinovaArm")
        test = ifaces.RoboCompKinovaArm.TJoint()
        print(f"Testing RoboCompKinovaArm.TJoints from ifaces.RoboCompKinovaArm")
        test = ifaces.RoboCompKinovaArm.TJoints()
        print(f"Testing RoboCompKinovaArm.TJointSpeeds from ifaces.RoboCompKinovaArm")
        test = ifaces.RoboCompKinovaArm.TJointSpeeds()
        print(f"Testing RoboCompKinovaArm.TJointAngles from ifaces.RoboCompKinovaArm")
        test = ifaces.RoboCompKinovaArm.TJointAngles()
        print(f"Testing RoboCompOmniRobot.TMechParams from ifaces.RoboCompOmniRobot")
        test = ifaces.RoboCompOmniRobot.TMechParams()
        print(f"Testing RoboCompJoystickAdapter.AxisParams from ifaces.RoboCompJoystickAdapter")
        test = ifaces.RoboCompJoystickAdapter.AxisParams()
        print(f"Testing RoboCompJoystickAdapter.ButtonParams from ifaces.RoboCompJoystickAdapter")
        test = ifaces.RoboCompJoystickAdapter.ButtonParams()
        print(f"Testing RoboCompJoystickAdapter.TData from ifaces.RoboCompJoystickAdapter")
        test = ifaces.RoboCompJoystickAdapter.TData()
        QTimer.singleShot(200, QApplication.instance().quit)



    # =============== Methods for Component SubscribesTo ================
    # ===================================================================

    #
    # SUBSCRIPTION to newFullPose method from FullPoseEstimationPub interface
    #
    def FullPoseEstimationPub_newFullPose(self, pose):
        if not self.useRTPose:
            #Change to  ros coordinates
            self.pose = np.array([pose.x*SCALE, pose.y*SCALE, pose.z*SCALE, pose.rx, pose.ry, pose.rz+1.57])
            # print(f"\rNew pose X:{self.pose[0]:.2f} | Y:{self.pose[1]:.2f} | Z:{self.pose[2]:.2f} | Roll:{self.pose[3]:.2f} | Pitch:{self.pose[4]:.2f} | Yaw:{self.pose[5]:.2f}", end="")


    #
    # SUBSCRIPTION to sendData method from JoystickAdapter interface
    #
    def JoystickAdapter_sendData(self, data):

        #
        # write your CODE here
        #
        pass


    # ===================================================================
    # ===================================================================



    ######################
    # From the RoboCompKinovaArm you can call this methods:
    # RoboCompKinovaArm.bool self.kinovaarm_proxy.closeGripper()
    # RoboCompKinovaArm.TPose self.kinovaarm_proxy.getCenterOfTool(ArmJoints referencedTo)
    # RoboCompKinovaArm.TGripper self.kinovaarm_proxy.getGripperState()
    # RoboCompKinovaArm.TJoints self.kinovaarm_proxy.getJointsState()
    # RoboCompKinovaArm.TToolInfo self.kinovaarm_proxy.getToolInfo()
    # RoboCompKinovaArm.void self.kinovaarm_proxy.moveJointsWithAngle(TJointAngles angles)
    # RoboCompKinovaArm.void self.kinovaarm_proxy.moveJointsWithSpeed(TJointSpeeds speeds)
    # RoboCompKinovaArm.void self.kinovaarm_proxy.openGripper()
    # RoboCompKinovaArm.void self.kinovaarm_proxy.setCenterOfTool(TPose pose, ArmJoints referencedTo)
    # RoboCompKinovaArm.bool self.kinovaarm_proxy.setGripperPos(float pos)

    ######################
    # From the RoboCompKinovaArm you can use this types:
    # ifaces.RoboCompKinovaArm.TPose
    # ifaces.RoboCompKinovaArm.TAxis
    # ifaces.RoboCompKinovaArm.TToolInfo
    # ifaces.RoboCompKinovaArm.TGripper
    # ifaces.RoboCompKinovaArm.TJoint
    # ifaces.RoboCompKinovaArm.TJoints
    # ifaces.RoboCompKinovaArm.TJointSpeeds
    # ifaces.RoboCompKinovaArm.TJointAngles

    ######################
    # From the RoboCompKinovaArm you can call this methods:
    # RoboCompKinovaArm.bool self.kinovaarm1_proxy.closeGripper()
    # RoboCompKinovaArm.TPose self.kinovaarm1_proxy.getCenterOfTool(ArmJoints referencedTo)
    # RoboCompKinovaArm.TGripper self.kinovaarm1_proxy.getGripperState()
    # RoboCompKinovaArm.TJoints self.kinovaarm1_proxy.getJointsState()
    # RoboCompKinovaArm.TToolInfo self.kinovaarm1_proxy.getToolInfo()
    # RoboCompKinovaArm.void self.kinovaarm1_proxy.moveJointsWithAngle(TJointAngles angles)
    # RoboCompKinovaArm.void self.kinovaarm1_proxy.moveJointsWithSpeed(TJointSpeeds speeds)
    # RoboCompKinovaArm.void self.kinovaarm1_proxy.openGripper()
    # RoboCompKinovaArm.void self.kinovaarm1_proxy.setCenterOfTool(TPose pose, ArmJoints referencedTo)
    # RoboCompKinovaArm.bool self.kinovaarm1_proxy.setGripperPos(float pos)

    ######################
    # From the RoboCompKinovaArm you can use this types:
    # ifaces.RoboCompKinovaArm.TPose
    # ifaces.RoboCompKinovaArm.TAxis
    # ifaces.RoboCompKinovaArm.TToolInfo
    # ifaces.RoboCompKinovaArm.TGripper
    # ifaces.RoboCompKinovaArm.TJoint
    # ifaces.RoboCompKinovaArm.TJoints
    # ifaces.RoboCompKinovaArm.TJointSpeeds
    # ifaces.RoboCompKinovaArm.TJointAngles

    ######################
    # From the RoboCompOmniRobot you can call this methods:
    # RoboCompOmniRobot.void self.omnirobot_proxy.correctOdometer(int x, int z, float alpha)
    # RoboCompOmniRobot.void self.omnirobot_proxy.getBasePose(int x, int z, float alpha)
    # RoboCompOmniRobot.void self.omnirobot_proxy.getBaseState(RoboCompGenericBase.TBaseState state)
    # RoboCompOmniRobot.void self.omnirobot_proxy.resetOdometer()
    # RoboCompOmniRobot.void self.omnirobot_proxy.setOdometer(RoboCompGenericBase.TBaseState state)
    # RoboCompOmniRobot.void self.omnirobot_proxy.setOdometerPose(int x, int z, float alpha)
    # RoboCompOmniRobot.void self.omnirobot_proxy.setSpeedBase(float advx, float advz, float rot)
    # RoboCompOmniRobot.void self.omnirobot_proxy.stopBase()

    ######################
    # From the RoboCompOmniRobot you can use this types:
    # ifaces.RoboCompOmniRobot.TMechParams

    ######################
    # From the RoboCompJoystickAdapter you can use this types:
    # ifaces.RoboCompJoystickAdapter.AxisParams
    # ifaces.RoboCompJoystickAdapter.ButtonParams
    # ifaces.RoboCompJoystickAdapter.TData



    # =============== DSR SLOTS  ================
    # =============================================

    def update_edge(self, fr: int, to: int, type: str):
        # console.print(f"UPDATE EDGE: {fr} to {to}", type, style='green')
        if self.useRTPose:
            room = self.g.get_node(fr)
            if room is not None:
                if room.name == "room_0" and to == ROBOT_DSR[1] and type == "RT":
                    try:
                        edge = self.rt.get_edge_RT(room, to)
                        if edge is not None:
                            index_new = np.argmax(edge.attrs["rt_timestamps"].value)*3
                            rot = edge.attrs["rt_rotation_euler_xyz"].value[index_new:index_new+3]
                            pose = edge.attrs["rt_translation"].value[index_new:index_new+3]
                            self.pose = np.array([pose[0]*SCALE, pose[1]*SCALE, pose[2]*SCALE, rot[0], rot[1], rot[2]+1.57])

                            # print(f"\rNew {index_new} pose X:{self.pose[0]:.2f} | Y:{self.pose[1]:.2f} | Z:{self.pose[2]:.2f} | Roll:{self.pose[3]:.2f} | Pitch:{self.pose[4]:.2f} | Yaw:{self.pose[5]:.2f}", end="")
                    except Exception as e:
                        print(f"Error procesando edge: {e}")

                        
        if to == ROBOT_DSR[1] and type == "TARGET":
            edge = self.g.get_edge(fr, to, "TARGET")
            if edge is not None:
                pose = edge.attrs["rt_translation"].value
                rot = edge.attrs["rt_rotation_euler_xyz"].value
                self.change_target(rot=rot, translate=pose)
                self.targetNode = fr

    def delete_edge(self, fr: int, to: int, type: str):
        # print(f"DELETE EDGE: {fr} to {to}")
        if fr == ROBOT_DSR[1] and to == self.targetNode and type == "TARGET":
            self.targetNode = None


