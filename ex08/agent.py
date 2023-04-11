import random
from dataclasses import dataclass
from typing import Sequence
import numpy as np
from typing import Optional

from commonroad.scenario.lanelet import LaneletNetwork
from dg_commons import PlayerName
from dg_commons.sim.goals import PlanningGoal
from dg_commons.sim import SimObservations, InitSimObservations
from dg_commons.sim.agents import Agent
from dg_commons.sim.models.obstacles import StaticObstacle
from dg_commons.sim.models.vehicle import VehicleCommands
from dg_commons.sim.models.vehicle_structures import VehicleGeometry
from dg_commons.sim.models.vehicle_utils import VehicleParameters
# controller
from dg_commons.controllers.speed import SpeedBehavior, SpeedController
from dg_commons.controllers.steer import SteerController
from dg_commons.controllers.pid import PIDParam
# import rrt
from pdm4ar.exercises.ex08.rrt import RRT
from geometry import SE2value, angle_from_SE2, translation_angle_from_SE2
from dg_commons.geo import SE2_apply_T2
from geometry import SE2_from_xytheta, SE2value, translation_from_SE2
#from dg_commons.maps import DgLanelet
#from dg_commons.controllers.pure_pursuit import PurePursuit, PurePursuitParam


class PurePursuitController:
    """ PurePursuit Controller
        Returns: delta_ref """
    def __init__(self, waypoints, lookahead, goal):
        self.waypoints = waypoints
        self.current_waypoint_idx = 0
        self.lookahead = lookahead
        self.index = None
        self.goal = goal
        self.lookead_minmax = (3,8)
    
        
    def find_goal_point(self, pos):
        lookahead = self.lookahead
        #print("lookahead", lookahead)
        min_dist = float('inf')
        closest_waypoint_idx = None
        for i, waypoint in enumerate(self.waypoints):
            dist = np.linalg.norm(np.array(waypoint) - np.array(pos))
            if dist < min_dist:
                min_dist = dist
                closest_waypoint_idx = i
        
        goal_point = self.goal
        for i in range(closest_waypoint_idx, len(self.waypoints)):
            waypoint = self.waypoints[i]
            self.index = i
            dist = np.linalg.norm(np.array(waypoint) - np.array(pos))
            if dist > lookahead:
                goal_point = waypoint
                break
        
        return goal_point
        
    
    def get_steering_angle(self, pos, pose, psi_ref):
            """
            :return: float the desired wheel angle
            """
            goal_point = self.find_goal_point(pos=pos)
            theta = angle_from_SE2(pose)
            rear_axle = SE2_apply_T2(pose, np.array([-1.2, 0]))
            # transform goal point to SE2 value
            if goal_point is not None:
                goal_point = SE2_from_xytheta([goal_point[0], goal_point[1], psi_ref[self.index-1]])
                #print("psi_ref of goal point", psi_ref[self.index-1])
            angle = psi_ref[self.index-1]
            p_goal, theta_goal = translation_angle_from_SE2(goal_point)
            alpha = np.arctan2(p_goal[1] - rear_axle[1], p_goal[0] - rear_axle[0]) - theta
            radius = self.lookahead / (2 * np.sin(alpha))
            delta_ref = np.arctan(3.5 / radius)
            delta_ref = np.clip(delta_ref, -1, 1)
            return delta_ref, angle

class Pdm4arAgent(Agent):
    """This is the PDM4AR agent.
    Do *NOT* modify the naming of the existing methods and the input/output types.
    Feel free to add additional methods, objects and functions that help you to solve the task"""

    def __init__(self,
                 sg: VehicleGeometry,
                 sp: VehicleParameters
                 ):
        self.sg = sg
        self.sp = sp
        self.name: PlayerName = None
        self.goal: PlanningGoal = None
        self.lanelet_network: LaneletNetwork = None
        self.static_obstacles: Sequence[StaticObstacle] = None
        # controller
        self.speed_controller = SpeedController.from_vehicle_params(model_param=self.sp) # initialize PID controller for speed
        self.speed_behavior: Optional[SpeedBehavior] = None # reference speed
        self.steer_controller = SteerController.from_vehicle_params(vehicle_param=self.sp)
        self.PID_params = PIDParam
        self.PID_params.kP = 1.3 #1.4?
        self.PID_params.kI = 0
        self.PID_params.kD = 1.7
        self.PID_params.setpoint_minmax = (-1.0,1.0)
        self.PID_params.output_minmax = (-0.5,0.5)
        self.steer_controller.params = self.PID_params
        self.lookahead = 3.2
        self.slow_down = 0.8
        #self.pure_pursuit: PurePursuit = PurePursuit.from_model_geometry(self.sg)
        #self.pure_pursuit_params = PurePursuitParam
        #self.pure_pursuit_params.look_ahead_minmax = (3,4)
        #self.pure_pursuit.param = self.pure_pursuit_params

    
    def on_episode_init(self, init_obs: InitSimObservations):
        """This method is called by the simulator at the beginning of each episode."""
        self.name = init_obs.my_name
        self.goal = init_obs.goal
        self.lanelet_network = init_obs.dg_scenario.lanelet_network
        self.static_obstacles = list(init_obs.dg_scenario.static_obstacles.values())
        # controller
        self.speed_behavior: SpeedBehavior = SpeedBehavior()
        self.speed_behavior.my_name = self.name
        self.pure_pursuit = None

        # RRT
        self.graph = None
        self.path = None
        self.success = False
        self.smoothed_path = None
        self.delta_ref = 0
        self.psi_ref = None

        # Multi-player


        
    def get_commands(self, sim_obs: SimObservations) -> VehicleCommands:
        """ This method is called by the simulator at each time step.
        For instance, this is how you can get your current state from the observations:
        my_current_state: VehicleState = sim_obs.players[self.name].state

        :param sim_obs:
        :return:
        """
        
        """ Vehicle params"""
        # Position
        agent = sim_obs.players[self.name]
        x = agent.state.x
        y = agent.state.y
        pos = (x,y)
        polygon = self.goal.goal
        end_pos_s = polygon.centroid # shapely point
        end_pos = (end_pos_s.x, end_pos_s.y) # convert to tuple
        # Orientation
        psi = agent.state.psi
        # Time
        t = float(sim_obs.time)
        # Static obstacles
        obstacles = self.static_obstacles
       
        """ Trajectory """
        # RRT graph
        RRT_ = RRT(startpos=pos, endpos=end_pos, obstacles=obstacles, lanelet_network = self.lanelet_network, buffer=2.5)
        while self.success is False:
            #print("buffersize", RRT_.buffer)
            self.graph, self.success = RRT_.generate_RRT()
            if self.success is False:
                RRT_.buffer -= 0.3

            #print("self.success", self.success)
            
            # Path from start to goal as list
            if self.success:
                self.path = RRT_.dijkstra(self.graph)
                self.smoothed_path = RRT_.smooth_path(self.path)
                #RRT_.plot(self.graph, path = self.smoothed_path)
            #else:
                #print("no path to goal found by RRT")  
        
        """ Multi-player """
        #print(sim_obs.players[self.name].occupancy)


        """ Steer controller """
        self._my_obs = sim_obs.players[self.name].state
        my_pose: SE2value = SE2_from_xytheta([self._my_obs.x, self._my_obs.y, self._my_obs.psi])
        
        if self.psi_ref == None:
            self.psi_ref = []
            for i in range(1,len(self.smoothed_path)):
                delta_x = self.smoothed_path[i][0] - self.smoothed_path[i-1][0]
                delta_y = self.smoothed_path[i][1] - self.smoothed_path[i-1][1]
                self.psi_ref.append(np.arctan2(delta_y, delta_x))    
    
        if self.pure_pursuit == None:
            self.pure_pursuit = PurePursuitController(self.smoothed_path, self.lookahead, end_pos)
        self.delta_ref, angle = self.pure_pursuit.get_steering_angle(pos, my_pose, self.psi_ref)

        # track delta_ref with PID controller
        self.steer_controller.update_measurement(measurement=self._my_obs.delta)
        self.steer_controller.update_reference(self.delta_ref)
        ddelta = self.steer_controller.get_control(t)   

        
        """ Speed controller """
        self.speed_behavior.update_observations(sim_obs.players)
        self.speed_controller.update_measurement(measurement=agent.state.vx)


        speed_ref, emergency = self.speed_behavior.get_speed_ref(t)
        if emergency:
            # Once the emergency kicks in the speed ref will always be 0
            self._emergency = True
            #print("emergency")
            speed_ref = 0.1

        #print("cruise control", self.speed_behavior.cruise_control(my_pose)) # relative pose = pose?
        if agent.state.vx > 8.5:
            speed_ref = 8.5
        # break before curves
        if abs(angle-psi) > 0.28:
            #print("break")
            speed_ref = 4.0

        self.speed_controller.update_reference(reference=speed_ref)
        acc = self.speed_controller.get_control(t) * self.slow_down

        return VehicleCommands(acc=acc, ddelta=ddelta)
