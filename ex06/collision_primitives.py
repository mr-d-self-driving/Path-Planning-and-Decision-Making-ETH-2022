from pdm4ar.exercises_def.ex06.structures import *
import triangle as tr
import numpy as np
from scipy import interpolate

class CollisionPrimitives:
    """
    Class of collusion primitives
    """

    @staticmethod
    def circle_point_collision(c: Circle, p: Point) -> bool:
        if (c.center.x - p.x)**2 + (c.center.y-p.y)**2 < c.radius**2:
            return True
        else:
            return False

    @staticmethod
    def triangle_point_collision(t: Triangle, p: Point) -> bool:
        areaOrig = abs( (t.v2.x-t.v1.x)*(t.v3.y-t.v1.y) - (t.v3.x-t.v1.x)*(t.v2.y-t.v1.y) )
        area1 =    abs( (t.v1.x-p.x)*(t.v2.y-p.y) - (t.v2.x-p.x)*(t.v1.y-p.y) )
        area2 =    abs( (t.v2.x-p.x)*(t.v3.y-p.y) - (t.v3.x-p.x)*(t.v2.y-p.y) )
        area3 =    abs( (t.v3.x-p.x)*(t.v1.y-p.y) - (t.v1.x-p.x)*(t.v3.y-p.y) )

        if round(area1+area2+area3) == round(areaOrig):
            return True
        else:
            return False

    @staticmethod
    def polygon_point_collision(poly: Polygon, p: Point) -> bool:
        # triangulate polygon
        v = []
        for i,point in enumerate(poly.vertices):
            v.append([point.x,point.y])
        t = tr.triangulate({'vertices':v})
        t_list = t['triangles'].tolist()
        # get triangle connectivity list)
        for i,triangle in enumerate(t_list):
            v1 = t['vertices'][triangle[0]]
            v2 = t['vertices'][triangle[1]]
            v3 = t['vertices'][triangle[2]]
            v1 = Point(v1[0],v1[1])
            v2 = Point(v2[0],v2[1])
            v3 = Point(v3[0],v3[1])
            tri = Triangle(v1,v2,v3)

            if CollisionPrimitives.triangle_point_collision(tri,p) is True:
                return True
        # point lies in none of the triangles
        return False

    @staticmethod
    def circle_segment_collision(c: Circle, segment: Segment) -> bool:
        # check if either start or end point of segment is in circle
        if CollisionPrimitives.circle_point_collision(c,segment.p1) is True or CollisionPrimitives.circle_point_collision(c,segment.p2) is True:
            return True
        # length of the segment
        length = np.sqrt((segment.p2.x-segment.p1.x)**2 +(segment.p2.y-segment.p1.y)**2)
        dot =  (((c.center.x - segment.p1.x)*(segment.p2.x-segment.p1.x)+(c.center.y - segment.p1.y)*(segment.p2.y-segment.p1.y))) / (length**2)
        # find the closest point to the circle on the line
        closest_x = segment.p1.x + (dot*(segment.p2.x-segment.p1.x))
        closest_y = segment.p1.y + (dot*(segment.p2.y-segment.p1.y))
        # check if closest point is on the line
        d1 = np.sqrt((c.center.x - segment.p1.x)**2 + (c.center.y - segment.p1.y)**2)
        d2 = np.sqrt((c.center.x - segment.p2.x)**2 + (c.center.y - segment.p2.y)**2)
        # point is not on line
        if round(d1+d2) != round(length):
            return False
        distX = closest_x - c.center.x
        distY = closest_y - c.center.y
        distance = np.sqrt( (distX*distX) + (distY*distY))
        if distance <= c.radius:
            return True
        else:
            return False
        
    @staticmethod
    def triangle_segment_collision(t: Triangle, segment: Segment) -> bool: # REWORKED
        # sample points on segment
        d = np.sqrt((segment.p2.x-segment.p1.x)**2 +(segment.p2.y-segment.p1.y)**2)
        for i in range(1,int(d*2)):
            xnew = segment.p1.x + (segment.p2.x-segment.p1.x)*(i/(d*2))
            ynew = segment.p1.y + (segment.p2.y-segment.p1.y)*(i/(d*2))
            if CollisionPrimitives.triangle_point_collision(t,Point(xnew,ynew)) is True:
                return True
        return False

    @staticmethod
    def polygon_segment_collision(p: Polygon, segment: Segment) -> bool:
        # sample points on segment
        d = np.sqrt((segment.p2.x-segment.p1.x)**2 +(segment.p2.y-segment.p1.y)**2)
        for i in range(1,int(d)+1):
            xnew = segment.p1.x + (segment.p2.x-segment.p1.x)*(i/d)
            ynew = segment.p1.y + (segment.p2.y-segment.p1.y)*(i/d)
            if CollisionPrimitives.polygon_point_collision(p,Point(xnew,ynew)) is True:
                return True
        return False

    @staticmethod
    def polygon_segment_collision_aabb(p: Polygon, segment: Segment) -> bool:
        aabb = CollisionPrimitives._poly_to_aabb(p)
        # sample points from segment
        d = np.sqrt((segment.p2.x-segment.p1.x)**2 +(segment.p2.y-segment.p1.y)**2)
        for i in range(1,int(d)+1):
            xnew = segment.p1.x + (segment.p2.x-segment.p1.x)*(i/d)
            ynew = segment.p1.y + (segment.p2.y-segment.p1.y)*(i/d)
            # check if points lie inside AABB
            if xnew > aabb.p_min.x and xnew < aabb.p_max.x and ynew > aabb.p_min.y and ynew < aabb.p_max.y:
                # point lies inside AABB
                # check if point lies in polygon, if not continue with the other points inside the AABB
                if CollisionPrimitives.polygon_point_collision(p,Point(xnew,ynew)) is False:
                    continue
                else:
                    return True
        # all points on the segment are checked -> point lies outside ABB
        return False

    @staticmethod
    def _poly_to_aabb(g: Polygon) -> AABB:
        # todo feel free to implement functions that upper-bound a shape with an
        #  AABB or simpler shapes for faster collision checks
        v = g.vertices
        minx ,miny = np.inf, np.inf
        maxx, maxy = -np.inf, -np.inf
        for i,point in enumerate(v):
            if point.x < minx:
                minx = point.x
            if point.y < miny:
                miny = point.y
            if point.x > maxx:
                maxx = point.x
            if point.y > maxy:
                maxy = point.y
        
        return AABB(p_min=Point(minx,miny), p_max=Point(maxx, maxy))
        #return AABB(p_min=Point(0, 0), p_max=Point(1, 1))
