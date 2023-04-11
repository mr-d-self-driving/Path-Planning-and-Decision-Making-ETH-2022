from typing import List
from dg_commons import SE2Transform
from pdm4ar.exercises.ex06.collision_primitives import CollisionPrimitives
from pdm4ar.exercises_def.ex06.structures import (
    Polygon,
    GeoPrimitive,
    Point,
    Segment,
    Circle,
    Triangle,
    Path,
)
import numpy as np
import random
from shapely.geometry import LineString
from shapely.strtree import STRtree
from shapely.geometry import Point as sPoint
from shapely.geometry import Polygon as sPolygon


##############################################################################################
############################# This is a helper function. #####################################
# Feel free to use this function or not

COLLISION_PRIMITIVES = {
    Point: {
        Circle: lambda x, y: CollisionPrimitives.circle_point_collision(y, x),
        Triangle: lambda x, y: CollisionPrimitives.triangle_point_collision(y, x),
        Polygon: lambda x, y: CollisionPrimitives.polygon_point_collision(y, x),
    },
    Segment: {
        Circle: lambda x, y: CollisionPrimitives.circle_segment_collision(y, x),
        Triangle: lambda x, y: CollisionPrimitives.triangle_segment_collision(y, x),
        Polygon: lambda x, y: CollisionPrimitives.polygon_segment_collision_aabb(y, x),
    },
    Triangle: {
        Point: CollisionPrimitives.triangle_point_collision,
        Segment: CollisionPrimitives.triangle_segment_collision,
    },
    Circle: {
        Point: CollisionPrimitives.circle_point_collision,
        Segment: CollisionPrimitives.circle_segment_collision,
    },
    Polygon: {
        Point: CollisionPrimitives.polygon_point_collision,
        Segment: CollisionPrimitives.polygon_segment_collision_aabb,
    },
}


def check_collision(p_1: GeoPrimitive, p_2: GeoPrimitive) -> bool:
    """
    Checks collision between 2 geometric primitives
    Note that this function only uses the functions that you implemented in CollisionPrimitives class.
        Parameters:
                p_1 (GeoPrimitive): Geometric Primitive
                p_w (GeoPrimitive): Geometric Primitive
    """
    assert type(p_1) in COLLISION_PRIMITIVES, "Collision primitive does not exist."
    assert (
        type(p_2) in COLLISION_PRIMITIVES[type(p_1)]
    ), "Collision primitive does not exist."

    collision_func = COLLISION_PRIMITIVES[type(p_1)][type(p_2)]

    return collision_func(p_1, p_2)


##############################################################################################


class CollisionChecker:
    """
    This class implements the collision check ability of a simple planner for a circular differential drive robot.

    Note that check_collision could be used to check collision between given GeoPrimitives
    check_collision function uses the functions that you implemented in CollisionPrimitives class.
    """

    def __init__(self):
        pass

    def path_collision_check(
        self, t: Path, r: float, obstacles: List[GeoPrimitive]
    ) -> List[int]:
        """
        Returns the indices of collided line segments.
        Note that index of first line segment is 0 and last line segment is len(t.waypoints)-1.

            Parameters:
                    t (Path): Path of circular differential drive robot
                    r (float): Radius of circular differential drive robot
                    obstacles (List[GeoPrimitive]): List of obstacles as GeoPrimitives
                    Please note that only Triangle, Circle and Polygon exist in this list
        """
        coll_segments = []
        # get segments as pairs
        segments = [[i,j] for i,j in zip(t.waypoints, t.waypoints[1:])]
    
        for j, seg in enumerate(segments):
            # sample points on each path segment as circle center
            d = np.sqrt((seg[1].x-seg[0].x)**2 +(seg[1].y-seg[0].y)**2)
            for i in range(1,int(d*2)):
                xnew = seg[0].x + (seg[1].x-seg[0].x)*(i/(d*2))
                ynew = seg[0].y + (seg[1].y-seg[0].y)*(i/(d*2))
                # create robot as set of points on circle circumference at each sampled point
                theta = random.random() * 2 * np.pi
                x = xnew + np.cos(theta) * r
                y = ynew + np.sin(theta) * r
                # at each position of the robot check for collision with obstacle 
                for n, obst in enumerate(obstacles):
                    if check_collision(obst,Point(x,y)) is True:
                    # if collision is detected append current path segment
                        if j not in coll_segments:
                            coll_segments.append(j)
        return coll_segments

    def path_collision_check_occupancy_grid(
        self, t: Path, r: float, obstacles: List[GeoPrimitive]
    ) -> List[int]:
        """
        Returns the indices of collided line segments.
        Note that index of first line segment is 0 and last line segment is len(t.waypoints)-1

        In this method, you will generate an occupancy grid of the given map.
        Then, occupancy grid will be used to check collisions.

            Parameters:
                    t (Path): Path of circular differential drive robot
                    r (float): Radius of circular differential drive robot
                    obstacles (List[GeoPrimitive]): List of obstacles as GeoPrimitives
                    Please note that only Triangle, Circle and Polygon exist in this list
        """
        coll_segments = []      
        
        # height (y-axis) and width (x-axis) of the path
        h = t.waypoints[len(t.waypoints)-1].y + int(r) + 1
        w = t.waypoints[len(t.waypoints)-1].x + int(r) + 1

        # grid scale 1:1
        occ_grid = np.zeros((h,w)) # shape (155,130)

        for x in range(occ_grid.shape[1]):
            for y in range(occ_grid.shape[0]):
                # sample 4 points around (x,y)
                xnew = np.linspace(x-0.9,x+0.9,2)
                ynew = np.linspace(y-0.9,y+0.9,2)
                # check for collision 
                for i,obst in enumerate(obstacles):
                    for i in range(len(xnew)):                 
                        if check_collision(Point(xnew[i],ynew[i]), obst):
                            occ_grid[y,x] = 1
                            break
        
        # get segments as pairs
        segments = [[i,j] for i,j in zip(t.waypoints, t.waypoints[1:])]
    
        for j, seg in enumerate(segments):
            # sample points on each path segment as circle center
            d = np.sqrt((seg[1].x-seg[0].x)**2 +(seg[1].y-seg[0].y)**2)
            for i in range(1,int(d*2)):
                xnew = seg[0].x + (seg[1].x-seg[0].x)*(i/(d*2))
                ynew = seg[0].y + (seg[1].y-seg[0].y)*(i/(d*2))
                # create robot as set of points on circle circumference at each sampled point
                theta = random.random() * 2 * np.pi
                x = xnew + np.cos(theta) * r
                y = ynew + np.sin(theta) * r
                # at each position of the robot check if occ_grid == 1
                if occ_grid[int(y),int(x)] == 1:
                    # if collision is detected append current path segment
                    if j not in coll_segments:
                        coll_segments.append(j)

        return coll_segments
    
    
    
    def path_collision_check_r_tree(
        self, t: Path, r: float, obstacles: List[GeoPrimitive]
    ) -> List[int]:
        """
        Returns the indices of collided line segments.
        Note that index of first line segment is 0 and last line segment is len(t.waypoints)-1

        In this method, you will build an R-Tree of the given obstacles.
        You are free to implement your own R-Tree or you could use STRTree of shapely module.

            Parameters:
                    t (Path): Path of circular differential drive robot
                    r (float): Radius of circular differential drive robot
                    obstacles (List[GeoPrimitive]): List of obstacles as GeoPrimitives
                    Please note that only Triangle, Circle and Polygon exist in this list
        """
        # list for shapely geometries
        tree_prep = []
        # list for colliding segments
        coll_segments = []

        # transform obstacles to shapely geometries
        for i,obst in enumerate(obstacles):
            if isinstance(obst, Polygon):
                poly = [sPoint(point.x, point.y) for point in obst.vertices]
                tree_prep.append(sPolygon(poly))
            if isinstance(obst, Triangle):
                tri = [sPoint(obst.v1.x,obst.v1.y),sPoint(obst.v2.x,obst.v2.y),sPoint(obst.v3.x,obst.v3.y)]
                tree_prep.append(sPolygon(tri))
            if isinstance(obst, Circle):
                circle = sPoint(obst.center.x, obst.center.y).buffer(r)
                tree_prep.append(circle)
        
        # create R-tree from shapely geometries
        tree = STRtree(tree_prep)

        # get segments as pairs
        segments = [[i,j] for i,j in zip(t.waypoints, t.waypoints[1:])]
    
        for j, seg in enumerate(segments):
            # sample points on each path segment as circle center
            d = np.sqrt((seg[1].x-seg[0].x)**2 +(seg[1].y-seg[0].y)**2)
            for i in range(1,int(d*2)):
                xnew = seg[0].x + (seg[1].x-seg[0].x)*(i/(d*2))
                ynew = seg[0].y + (seg[1].y-seg[0].y)*(i/(d*2))
                #robot = sPoint(xnew,ynew).buffer(r)
                # create robot as set of points on circle circumference at each sampled point
                theta = random.random() * 2 * np.pi
                x = xnew + np.cos(theta) * r
                y = ynew + np.sin(theta) * r
                # at each position of the robot check for collision with obstacle
                indices = tree.query(sPoint(x,y))
                # if collision is detected add segment to collision list
                if indices != []:
                    if j not in coll_segments:
                        coll_segments.append(j)
        return coll_segments

    def collision_check_robot_frame(
        self,
        r: float,
        current_pose: SE2Transform,
        next_pose: SE2Transform,
        observed_obstacles: List[GeoPrimitive],
    ) -> bool:
        """
        Returns there exists a collision or not during the movement of a circular differential drive robot until its next pose.

            Parameters:
                    r (float): Radius of circular differential drive robot
                    current_pose (SE2Transform): Current pose of the circular differential drive robot
                    next_pose (SE2Transform): Next pose of the circular differential drive robot
                    observed_obstacles (List[GeoPrimitive]): List of obstacles as GeoPrimitives in robot frame
                    Please note that only Triangle, Circle and Polygon exist in this list
        """
    
        # check collision of segment btw current and next pose (global SE2 transform) and obstacle, (robot frame -> global SE2 transform)
        seg = Segment(Point(current_pose.p[0], current_pose.p[1]),Point(next_pose.p[0], next_pose.p[1]))
    
        # sample points on each path segment as circle center
        d = np.sqrt((seg.p2.x-seg.p1.x)**2 +(seg.p2.y-seg.p1.y)**2)
        for i in range(1,int(d*2)):
            xnew = seg.p1.x + (seg.p2.x-seg.p1.x)*(i/(d*2))
            ynew = seg.p1.y + (seg.p2.y-seg.p1.y)*(i/(d*2))
            # create robot as set of points on circle circumference at each sampled point
            theta = random.random() * 2 * np.pi
            x = xnew + np.cos(theta) * r
            y = ynew + np.sin(theta) * r
            # at each position of the robot check for collision with obstacle 
            for i, obst in enumerate(observed_obstacles):
                obst_global = obst.apply_SE2transform(current_pose.as_SE2())
                if check_collision(obst_global, Point(x,y)) is True:
                    return True
        return False
    
    @staticmethod
    def nearest(S,x):
        """ returns closest point s in S to x"""
        distances = []
        if S == []:
            return None
        else:
            for i in range(len(S)):
                dist = np.linalg.norm(np.array(x)-np.array(S[i]))
                distances.append(dist)
            s = np.argmin(distances)
            return S[s]

    @staticmethod
    def setDistance(S,x,r):
        """ returns the minimum distance of x to the a point s in set S
        :param S: obstacle set X_obs """
        # minimum distance of x to obstacle set
        if S == []:
            return None
        
        dist_obs = []
        dist_free = []
        sx = sPoint(x[0],x[1])

        for i,obst in enumerate(S):
            # convert obstacles to shapely geoms
            if isinstance(obst, Polygon):
                poly = sPolygon([sPoint(point.x, point.y) for point in obst.vertices])
                dist_obs.append(poly.distance(sx))
                dist_free.append(poly.exterior.distance(sx))
            if isinstance(obst, Triangle):
                tri = sPolygon([sPoint(obst.v1.x,obst.v1.y),sPoint(obst.v2.x,obst.v2.y),sPoint(obst.v3.x,obst.v3.y)])
                dist_obs.append(tri.distance(sx))
                dist_free.append(tri.exterior.distance(sx))
            if isinstance(obst, Circle):
                circle = sPoint(obst.center.x, obst.center.y).buffer(r)
                dist_obs.append(circle.distance(sx))
                dist_free.append(circle.distance(sx))
        min_dist_obs = np.min(dist_obs)
        min_dist_free = np.min(dist_free)
        return min_dist_obs, min_dist_free
    

    def path_collision_check_safety_certificate(
        self, t: Path, r: float, obstacles: List[GeoPrimitive]
    ) -> List[int]:
        """
        Returns the indices of collided line segments.
        Note that index of first line segment is 0 and last line segment is len(t.waypoints)-1

        In this method, you will implement the safety certificates procedure for collision checking.
        You are free to use shapely to calculate distance between a point and a GoePrimitive.
        For more information, please check Algorithm 1 inside the following paper:
        https://journals.sagepub.com/doi/full/10.1177/0278364915625345.

            Parameters:
                    t (Path): Path of circular differential drive robot
                    r (float): Radius of circular differential drive robot
                    obstacles (List[GeoPrimitive]): List of obstacles as GeoPrimitives
                    Please note that only Triangle, Circle and Polygon exist in this list
        """
        coll_segments = []
        # points for which collision check has detected no collision
        S_free = []
        # points for which collision check has detected collision
        S_obs = []
        # stores lower bound on minimum distances to obstacles for all points in S_free 
        # stores lower bound on minimum distances to the free space (obstacle boundary) for all points in S_obs
        Dist = {}
        
        # sample points xq on each path segment
        segments = [[i,j] for i,j in zip(t.waypoints, t.waypoints[1:])]
        for j, seg in enumerate(segments):
            d = np.sqrt((seg[1].x-seg[0].x)**2 +(seg[1].y-seg[0].y)**2)
            for i in range(1,int(d*2)):
                xnew = seg[0].x + (seg[1].x-seg[0].x)*(i/(d*2))
                ynew = seg[0].y + (seg[1].y-seg[0].y)*(i/(d*2))
                # create robot as set of points on circle circumference at each sampled point
                theta = random.random() * 2 * np.pi
                x = xnew + np.cos(theta) * r
                y = ynew + np.sin(theta) * r
                xq = [x,y]

                # register each sampled point xq into S_free or S_obs
                x_free = CollisionChecker.nearest(S_free,xq)
                x_obs = CollisionChecker.nearest(S_obs,xq)
                print("x_free",x_free)
                # Don't need that check because if no collision is detected i need to check the other samples
                #if x_free is not None:
                #    if np.linalg.norm(np.array(xq) - np.array(x_free)) <= Dist[(x_free[0],x_free[1])]:
                #        # xq collision-free
                #        pass
                if x_obs is not None:
                    if np.linalg.norm(np.array(xq) - np.array(x_obs)) <= Dist[(x_obs[0],x_obs[1])]:
                        # xq in collision
                        if j not in coll_segments:
                            coll_segments.append(j)
                            break
                        
                # if no certificate information exists
                dist_to_obs, dist_to_free = CollisionChecker.setDistance(obstacles,xq,r)
                if dist_to_obs > 0:
                    # xq collision-free
                    S_free.append(xq)
                    Dist[(xq[0],xq[1])] = dist_to_obs 
                else:
                    # xq in collision
                    S_obs.append(xq)
                    Dist[(xq[0],xq[1])] = dist_to_free
                    if j not in coll_segments:
                        coll_segments.append(j)
                        break
        
        return coll_segments
