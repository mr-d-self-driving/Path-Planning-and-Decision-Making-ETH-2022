from typing import Sequence

from dg_commons import SE2Transform

from pdm4ar.exercises.ex05.structures import *
from pdm4ar.exercises_def.ex05.utils import extract_path_points
import numpy as np
from math import *


class PathPlanner(ABC):
    @abstractmethod
    def compute_path(self, start: SE2Transform, end: SE2Transform) -> Sequence[SE2Transform]:
        pass


class Dubins(PathPlanner):
    def __init__(self, params: DubinsParam):
        self.params = params

    def compute_path(self, start: SE2Transform, end: SE2Transform) -> List[SE2Transform]:
        """ Generates an optimal Dubins path between start and end configuration

        :param start: the start configuration of the car (x,y,theta)
        :param end: the end configuration of the car (x,y,theta)

        :return: a List[SE2Transform] of configurations in the optimal path the car needs to follow
        """
        path = calculate_dubins_path(start_config=start, end_config=end, radius=self.params.min_radius)
        se2_list = extract_path_points(path)
        return se2_list


class ReedsShepp(PathPlanner):
    def __init__(self, params: DubinsParam):
        self.params = params

    def compute_path(self, start: SE2Transform, end: SE2Transform) -> Sequence[SE2Transform]:
        """ Generates a Reeds-Shepp *inspired* optimal path between start and end configuration

        :param start: the start configuration of the car (x,y,theta)
        :param end: the end configuration of the car (x,y,theta)

        :return: a List[SE2Transform] of configurations in the optimal path the car needs to follow 
        """
        path = calculate_reeds_shepp_path(start_config=start, end_config=end, radius=self.params.min_radius)
        se2_list = extract_path_points(path)
        return se2_list


def calculate_car_turning_radius(wheel_base: float, max_steering_angle: float) -> DubinsParam:
    # TODO implement here your solution
    min_radius = wheel_base / np.tan(max_steering_angle)
    return DubinsParam(min_radius)


def calculate_turning_circles(current_config: SE2Transform, radius: float) -> TurningCircle:
    # TODO implement here your solution

    # reworked
    # current config = ([x,y],theta)
    pos = current_config.p
    theta = 0

    # calculate center of the right circle
    x_right = pos[0] + (radius*np.cos(current_config.theta-np.pi/2))
    y_right = pos[1] + (radius*np.sin(current_config.theta-np.pi/2))

    # calculate center of left circle
    x_left = pos[0] - (radius*np.cos(current_config.theta-np.pi/2))
    y_left = pos[1] - (radius*np.sin(current_config.theta-np.pi/2))

    left_circle = Curve.create_circle(center=SE2Transform([x_left,y_left], theta), config_on_circle=current_config,
                                       radius=radius, curve_type=DubinsSegmentType.LEFT) 
    right_circle = Curve.create_circle(center=SE2Transform([x_right,y_right],theta), config_on_circle=current_config,
                                       radius=radius, curve_type=DubinsSegmentType.RIGHT)  

    return TurningCircle(left=left_circle, right=right_circle)


def calculate_tangent_btw_circles(circle_start: Curve, circle_end: Curve) -> List[Line]:
    # TODO implement here your solution

    # calculate euclidean distance between two circle centers
    x1 = circle_start.center.p[0]
    y1 = circle_start.center.p[1]
    x2 = circle_end.center.p[0]
    y2 = circle_end.center.p[1]
    distance = np.sqrt((x2-x1)**2+(y2-y1)**2)

    # get start and end circle radius
    r1 = circle_start.radius
    r2 = circle_end.radius

    # draw vector C between centers of both circles
    cx = (x2-x1)/distance
    cy = (y2-y1)/distance

    ### special cases: no tangent exists ###

    # 1. circles have the same center
    if circle_start.center == circle_end.center:
        return []
    # 2. circles are intersecting REWORKED
    #if distance <r1+r2:
    #    return []

    ### outer tangents ###

    if circle_start.type == DubinsSegmentType.RIGHT and circle_end.type == DubinsSegmentType.RIGHT:

        # calculate n that is perpendicular vector to tangent
        c = (r1-r2)/distance

        h = np.sqrt(max(0.0, 1.0 - c*c))
        nx = cx*c - h*cy
        ny = cy*c + h*cx

        # tangent points
        p1 = [x1 + r1 * nx, y1 + r1 * ny]
        p2 = [x2 + r2 * nx, y2 + r2 * ny]

        # angle
        theta = np.arctan2(p2[1]-p1[1],p2[0]-p1[0])
        
        # tangent line
        tangent = Line(SE2Transform(p1, theta),SE2Transform(p2,theta))

        return [tangent]

    if circle_start.type == DubinsSegmentType.LEFT and circle_end.type == DubinsSegmentType.LEFT:

        # calculate n that is perpendicular vector to tangent
        c = (r1-r2)/distance

        h = np.sqrt(max(0.0, 1.0 - c*c))
        nx = cx*c + h*cy
        ny = cy*c - h*cx

        # tangent points
        p1 = [x1 + r1 * nx, y1 + r1 * ny]
        p2 = [x2 + r2 * nx, y2 + r2 * ny]

        # angle
        theta = np.arctan2(p2[1]-p1[1],p2[0]-p1[0])
        
        # tangent line
        tangent = Line(SE2Transform(p1, theta),SE2Transform(p2,theta))

        return [tangent]

    ### inner tangents RL ###

    if circle_start.type == DubinsSegmentType.RIGHT and circle_end.type == DubinsSegmentType.LEFT:

        # calculate n that is perpendicular vector to tangent
        c = (r1+r2)/distance

        h = np.sqrt(max(0.0, 1.0 - c*c))
        nx = cx*c - h*cy
        ny = cy*c + h*cx

        # tangent points
        p1 = [x1 + r1 * nx, y1 + r1 * ny]
        p2 = [x2 - r2 * nx, y2 - r2 * ny]

        # angle
        theta = np.arctan2(p2[1]-p1[1],p2[0]-p1[0])

        # tangent line
        tangent = Line(SE2Transform(p1, theta),SE2Transform(p2,theta))

        return [tangent]

    ### inner tangents LR ###

    if circle_start.type == DubinsSegmentType.LEFT and circle_end.type == DubinsSegmentType.RIGHT:

        # calculate n that is perpendicular vector to tangent
        c = (r1+r2)/distance
        h = np.sqrt(max(0.0, 1.0 - c*c))
        nx = cx*c + h*cy 
        ny = cy*c - h*cx

        # tangent points
        p1 = [x1 + r1 * nx, y1 + r1 * ny]
        p2 = [x2 - r2 * nx, y2 - r2 * ny]

        # angle
        theta = np.arctan2(p2[1]-p1[1],p2[0]-p1[0])

        # tangent line
        tangent = Line(SE2Transform(p1, theta),SE2Transform(p2, theta))

        return [tangent]


def calculate_circle_btw_circles(circle_start: Curve, circle_end: Curve, radius: float) -> Curve:
    """ param: circle_start: start curve
        param: circle_end: end curve
        param: radius: minimum turning radius
        returns: a turning circle with minimum radius that is tangent to start and end circle """
    
    # calculate euclidean distance between two circle centers
    x1 = circle_start.center.p[0]
    y1 = circle_start.center.p[1]
    x2 = circle_end.center.p[0]
    y2 = circle_end.center.p[1]
    distance = np.sqrt((x2-x1)**2+(y2-y1)**2)

    if circle_start.type == DubinsSegmentType.RIGHT and circle_end.type == DubinsSegmentType.RIGHT: # RLR
        # calculate angle 
        theta = acos(distance/(4*radius)) #domain
        # circle center of circle_start and circle_end
        p1 = circle_start.center.p
        p2 = circle_end.center.p
        v1 = p2-p1
        offset = np.arctan2(v1[1],v1[0])
        # absolute amount of rotation
        theta = offset-theta   # RLR substract
        # compute center of tangent circle
        p3 = (x1+2*radius*np.cos(theta), y1+2*radius*np.sin(theta))
        print("p3",p3)
        # compute tangent point tan_p1
        v2_p1 = p1-p3
        mag_p1 = np.sqrt(v2_p1[0]**2 + v2_p1[1]**2)
        v2_p1 = v2_p1/mag_p1*radius
        tan_p1 = p3+v2_p1
        # compute tangent point tan_p2
        v2_p2 = p1-p2
        mag_p2 = np.sqrt(v2_p2[0]**2 + v2_p2[1]**2)
        v2_p2 = v2_p2/mag_p2*radius
        tan_p2 = p3+v2_p2

        # angle
        d = circle_end.center.p - circle_start.center.p
        theta = np.arctan2(d[1],d[0])
        theta_p1 = theta-(np.arccos(distance/2/(2*radius))+np.pi/2)
        theta_p2 = theta+(np.arccos(distance/2/(2*radius))+np.pi/2)
        
        # calculate arc_angle
        arc_angle = -(theta_p1 - theta_p2)

        #calculate turning circle left
        left_circle = Curve(start_config=SE2Transform(tan_p1, theta_p1), 
                            end_config=SE2Transform(tan_p2, theta_p2), 
                            center = SE2Transform(p3, theta=0), 
                            radius =radius,
                            curve_type=DubinsSegmentType.LEFT,
                            arc_angle=arc_angle)
        return left_circle

    else: # LRL
        # calculate angle 
        theta = np.arccos(distance/(4*radius))
        # circle center of circle_start and circle_end
        p1 = circle_start.center.p
        p2 = circle_end.center.p
        v1 = p2-p1
        offset = np.arctan2(v1[1],v1[0])
        # absolute amount of rotation
        theta += offset  # LRL add
        # compute center of tangent circle
        p3 = (x1+2*radius*np.cos(theta), y1+2*radius*np.sin(theta))
        # compute tangent point tan_p1
        v2_p1 = p1-p3
        mag_p1 = np.sqrt(v2_p1[0]**2 + v2_p1[1]**2)
        v2_p1 = v2_p1/mag_p1*radius
        tan_p1 = p3+v2_p1
        # compute tangent point tan_p2
        v2_p2 = p1-p2
        mag_p2 = np.sqrt(v2_p2[0]**2 + v2_p2[1]**2)
        v2_p2 = v2_p2/mag_p2*radius
        tan_p2 = p3+v2_p2

        # angle
        psi = np.arctan2(y2-y1, x2-x1)
        theta_p1 = psi+theta+np.pi/2
        theta_p2 = psi-np.pi/2-theta

        arc_angle = (theta_p1 - theta_p2)

        #calculate turning circle left
        right_circle = Curve(start_config=SE2Transform(tan_p1, theta_p1), 
                            end_config=SE2Transform(tan_p2, theta_p2), 
                            center = SE2Transform(p3, theta=0), 
                            radius =radius,
                            curve_type=DubinsSegmentType.RIGHT,
                            arc_angle=arc_angle)
        return right_circle



def calculate_dubins_path(start_config: SE2Transform, end_config: SE2Transform, radius: float) -> Path:
    # TODO implement here your solution
    # Please keep segments with zero length in the return list & return a valid dubins path!

    ### CSC Paths: LSR, RSL, LSL, RSR ###

    ### LSL ###
    start_circle = calculate_turning_circles(start_config, radius).left
    end_circle = calculate_turning_circles(end_config, radius).left

    tangent = calculate_tangent_btw_circles(start_circle, end_circle)
    tangent = tangent[0] # get line object

    arc_angle_1 = -(start_config.theta - tangent.start_config.theta)
    arc_angle_3 = -(tangent.end_config.theta-end_config.theta)

    seg1 = Curve(start_config=start_config, end_config=tangent.start_config, center = start_circle.center, radius =radius,curve_type=DubinsSegmentType.LEFT,arc_angle=arc_angle_1)
    seg2 = tangent
    seg3 = Curve(start_config = tangent.end_config, end_config = end_config, center = end_circle.center, radius = radius,curve_type=DubinsSegmentType.LEFT,arc_angle=arc_angle_3)

    LSL = [seg1,seg2,seg3]

    ### LSR ###
    LSR_start_circle = calculate_turning_circles(start_config, radius).left
    LSR_end_circle = calculate_turning_circles(end_config, radius).right

    LSR_tangent = calculate_tangent_btw_circles(LSR_start_circle, LSR_end_circle)
    LSR_tangent = LSR_tangent[0] # get line object

    LSR_arc_angle_1 = -(start_config.theta - LSR_tangent.start_config.theta)
    LSR_arc_angle_3 = (LSR_tangent.end_config.theta-end_config.theta)
    
    LSR_seg1 = Curve(start_config=start_config, end_config=LSR_tangent.start_config, center = LSR_start_circle.center, radius =radius,curve_type=DubinsSegmentType.LEFT,arc_angle=LSR_arc_angle_1)
    LSR_seg2 = LSR_tangent
    LSR_seg3 = Curve(start_config = LSR_tangent.end_config, end_config = end_config, center = LSR_end_circle.center, radius = radius,curve_type=DubinsSegmentType.RIGHT,arc_angle=LSR_arc_angle_3)

    LSR =  [LSR_seg1,LSR_seg2,LSR_seg3]

    ### RSR ###
    RSR_start_circle = calculate_turning_circles(start_config, radius).right
    RSR_end_circle = calculate_turning_circles(end_config, radius).right

    RSR_tangent = calculate_tangent_btw_circles(RSR_start_circle, RSR_end_circle)
    RSR_tangent = RSR_tangent[0] # get line object

    RSR_arc_angle_1 = (start_config.theta - RSR_tangent.start_config.theta)
    RSR_arc_angle_3 = (RSR_tangent.end_config.theta-end_config.theta)
    
    RSR_seg1 = Curve(start_config=start_config, end_config=RSR_tangent.start_config, center = RSR_start_circle.center, radius =radius,curve_type=DubinsSegmentType.RIGHT,arc_angle=RSR_arc_angle_1)
    RSR_seg2 = RSR_tangent
    RSR_seg3 = Curve(start_config = RSR_tangent.end_config, end_config = end_config, center = RSR_end_circle.center, radius = radius,curve_type=DubinsSegmentType.RIGHT,arc_angle=RSR_arc_angle_3)

    RSR =  [RSR_seg1,RSR_seg2,RSR_seg3]

    ### RSL ###
    RSL_start_circle = calculate_turning_circles(start_config, radius).right
    RSL_end_circle = calculate_turning_circles(end_config, radius).left

    RSL_tangent = calculate_tangent_btw_circles(RSL_start_circle, RSL_end_circle)
    RSL_tangent = RSL_tangent[0] # get line object

    RSL_arc_angle_1 = (start_config.theta - RSL_tangent.start_config.theta)
    RSL_arc_angle_3 = -(RSL_tangent.end_config.theta-end_config.theta)
    RSL_seg1 = Curve(start_config=start_config, end_config=RSL_tangent.start_config, center = RSL_start_circle.center, radius =radius,curve_type=DubinsSegmentType.RIGHT,arc_angle=RSL_arc_angle_1)
    RSL_seg2 = RSL_tangent
    RSL_seg3 = Curve(start_config = RSL_tangent.end_config, end_config = end_config, center = RSL_end_circle.center, radius = radius,curve_type=DubinsSegmentType.LEFT,arc_angle=RSL_arc_angle_3)

    RSL =  [RSL_seg1,RSL_seg2,RSL_seg3]

    ### CCC paths: RLR, LRL ###
    
    # calculate length of each path
    length = {"LSR":0.0,"RSL":0.0,"LSL":0.0,"RSR":0.0}
    for i in range(len(LSL)):
        length["LSR"] += LSR[i].length
        length["RSL"] += RSL[i].length
        length["LSL"] += LSL[i].length
        length["RSR"] += RSR[i].length
    shortest_length = min(length, key=length.get)
    if shortest_length == "LSR":
        return LSR
    if shortest_length == "RSL":
        return RSL
    if shortest_length == "LSL":
        return LSL
    if shortest_length == "RSR":
        return RSR

def calculate_reeds_shepp_path(start_config: SE2Transform, end_config: SE2Transform, radius: float) -> Path:
    # TODO implement here your solution
    # Please keep segments with zero length in the return list & return a valid dubins/reeds path!

    ### foward paths ###
    shortest_dubins = calculate_dubins_path(start_config=start_config,end_config=end_config,radius=radius)
    
    ### C⁻S⁻C⁻ paths: LSL,RSL,LSR,RSL ###
    ### LSL ###
    start_circle = calculate_turning_circles(start_config, radius).left
    end_circle = calculate_turning_circles(end_config, radius).left

    tangent = calculate_tangent_btw_circles(end_circle, start_circle)
    tangent = tangent[0] # get line object
    tangent.gear = Gear.REVERSE

    end = tangent.end_config
    start = tangent.start_config
    tangent.start_config = end
    tangent.end_config = start
    
    arc_angle_1 = -(start_config.theta - tangent.start_config.theta)
    arc_angle_3 = -(tangent.end_config.theta - end_config.theta)

    seg1 = Curve(start_config = start_config, end_config = tangent.start_config, center = start_circle.center, radius = radius,curve_type=DubinsSegmentType.LEFT,arc_angle=arc_angle_1, gear=Gear.REVERSE)
    seg2 = tangent
    seg3 = Curve(start_config=tangent.end_config, end_config=end_config, center = end_circle.center, radius =radius,curve_type=DubinsSegmentType.LEFT,arc_angle=arc_angle_3,gear=Gear.REVERSE)

    LSL = [seg1,seg2,seg3]

    ### LSR ###

    LSR_start_circle = calculate_turning_circles(start_config, radius).left
    LSR_end_circle = calculate_turning_circles(end_config, radius).right

    LSR_tangent = calculate_tangent_btw_circles(LSR_end_circle, LSR_start_circle)
    LSR_tangent = LSR_tangent[0] # get line object
    LSR_tangent.gear = Gear.REVERSE

    LSR_end = LSR_tangent.end_config
    LSR_start = LSR_tangent.start_config
    LSR_tangent.start_config = LSR_end
    LSR_tangent.end_config = LSR_start

    LSR_arc_angle_1 = -(start_config.theta - LSR_tangent.start_config.theta)
    LSR_arc_angle_3 = (LSR_tangent.end_config.theta-end_config.theta)
    
    LSR_seg1 = Curve(start_config=start_config, end_config=LSR_tangent.start_config, center = LSR_start_circle.center, radius =radius,curve_type=DubinsSegmentType.LEFT,arc_angle=LSR_arc_angle_1,gear=Gear.REVERSE)
    LSR_seg2 = LSR_tangent
    LSR_seg3 = Curve(start_config = LSR_tangent.end_config, end_config = end_config, center = LSR_end_circle.center, radius = radius,curve_type=DubinsSegmentType.RIGHT,arc_angle=LSR_arc_angle_3,gear=Gear.REVERSE)

    LSR =  [LSR_seg1,LSR_seg2,LSR_seg3]   

    ### RSR ###
    RSR_start_circle = calculate_turning_circles(start_config, radius).right
    RSR_end_circle = calculate_turning_circles(end_config, radius).right

    RSR_tangent = calculate_tangent_btw_circles(RSR_end_circle, RSR_start_circle)
    RSR_tangent = RSR_tangent[0] # get line object
    RSR_tangent.gear = Gear.REVERSE

    RSR_end = RSR_tangent.end_config
    RSR_start = RSR_tangent.start_config
    RSR_tangent.start_config = RSR_end
    RSR_tangent.end_config = RSR_start

    RSR_arc_angle_1 = (start_config.theta - RSR_tangent.start_config.theta)
    RSR_arc_angle_3 = (RSR_tangent.end_config.theta-end_config.theta)
    
    RSR_seg1 = Curve(start_config=start_config, end_config=RSR_tangent.start_config, center = RSR_start_circle.center, radius =radius,curve_type=DubinsSegmentType.RIGHT,arc_angle=RSR_arc_angle_1,gear=Gear.REVERSE)
    RSR_seg2 = RSR_tangent
    RSR_seg3 = Curve(start_config = RSR_tangent.end_config, end_config = end_config, center = RSR_end_circle.center, radius = radius,curve_type=DubinsSegmentType.RIGHT,arc_angle=RSR_arc_angle_3,gear=Gear.REVERSE)

    RSR =  [RSR_seg1,RSR_seg2,RSR_seg3]

    ### RSL ###
    RSL_start_circle = calculate_turning_circles(start_config, radius).right
    RSL_end_circle = calculate_turning_circles(end_config, radius).left

    RSL_tangent = calculate_tangent_btw_circles(RSL_end_circle, RSL_start_circle)
    RSL_tangent = RSL_tangent[0] # get line object
    RSL_tangent.gear = Gear.REVERSE

    RSL_end = RSL_tangent.end_config
    RSL_start = RSL_tangent.start_config
    RSL_tangent.start_config = RSL_end
    RSL_tangent.end_config = RSL_start

    RSL_arc_angle_1 = (start_config.theta - RSL_tangent.start_config.theta)
    RSL_arc_angle_3 = -(RSL_tangent.end_config.theta-end_config.theta)
    RSL_seg1 = Curve(start_config=start_config, end_config=RSL_tangent.start_config, center = RSL_start_circle.center, radius =radius,curve_type=DubinsSegmentType.RIGHT,arc_angle=RSL_arc_angle_1,gear = Gear.REVERSE)
    RSL_seg2 = RSL_tangent
    RSL_seg3 = Curve(start_config = RSL_tangent.end_config, end_config = end_config, center = RSL_end_circle.center, radius = radius,curve_type=DubinsSegmentType.LEFT,arc_angle=RSL_arc_angle_3,gear = Gear.REVERSE)

    RSL =  [RSL_seg1,RSL_seg2,RSL_seg3]

    ### RLR ###
    RLR_start_circle = calculate_turning_circles(start_config, radius).right
    RLR_end_circle = calculate_turning_circles(end_config, radius).right

    RLR_tangent_circle = calculate_circle_btw_circles(RLR_start_circle,RLR_end_circle,radius)
    RLR_arc_angle_1 = (start_config.theta-RLR_tangent_circle.start_config.theta)
    RLR_arc_angle_3 = (RLR_tangent_circle.end_config.theta-end_config.theta)
    RLR_seg1 = Curve(start_config=start_config,end_config=RLR_tangent_circle.start_config, center = start_circle.center, radius=radius, curve_type=DubinsSegmentType.RIGHT, arc_angle=RLR_arc_angle_1)
    RLR_seg2 = RLR_tangent_circle
    RLR_seg3 = Curve(start_config=RLR_tangent_circle.end_config,end_config=end_config, center = end_circle.center, radius=radius, curve_type=DubinsSegmentType.RIGHT, arc_angle=RLR_arc_angle_3)
    
    RLR = [RLR_seg1,RLR_seg2,RLR_seg3]
    #return RLR

    ### LRL ###
    LRL_start_circle = calculate_turning_circles(start_config, radius).left
    LRL_end_circle = calculate_turning_circles(end_config, radius).left

    LRL_tangent_circle = calculate_circle_btw_circles(LRL_start_circle,LRL_end_circle,radius)
    LRL_arc_angle_1 = -(start_config.theta-LRL_tangent_circle.start_config.theta)
    LRL_arc_angle_3 = -(LRL_tangent_circle.end_config.theta-end_config.theta)
    LRL_seg1 = Curve(start_config=start_config,end_config=LRL_tangent_circle.start_config, center = start_circle.center, radius=radius, curve_type=DubinsSegmentType.LEFT, arc_angle=LRL_arc_angle_1)
    LRL_seg2 = LRL_tangent_circle
    LRL_seg3 = Curve(start_config=LRL_tangent_circle.end_config,end_config=end_config, center = end_circle.center, radius=radius, curve_type=DubinsSegmentType.LEFT, arc_angle=LRL_arc_angle_3)
    
    LRL = [LRL_seg1,LRL_seg2,LRL_seg3]
    return LRL


    # calculate length of each path
    length = {"LSR":0.0,"RSL":0.0,"LSL":0.0,"RSR":0.0, "dubins": 0.0}
    for i in range(len(LSL)):
        length["LSR"] += LSR[i].length
        length["RSL"] += RSL[i].length
        length["LSL"] += LSL[i].length
        length["RSR"] += RSR[i].length
        length["dubins"] += shortest_dubins[i].length
    shortest_length = min(length, key=length.get)
    if shortest_length == "LSR":
        return LSR
    if shortest_length == "RSL":
        return RSL
    if shortest_length == "LSL":
        return LSL
    if shortest_length == "RSR":
        return RSR
    if shortest_length == "dubins":
        return shortest_dubins