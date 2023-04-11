import random

from shapely.geometry import LineString, LinearRing
from shapely.geometry import Point as sPoint
from shapely.geometry import Polygon as sPolygon
import numpy as np
import matplotlib.pyplot as plt
from collections import deque
from matplotlib import collections  as mc

class Graph:
    """ Define graph """
    def __init__(self, startpos, endpos):
        self.startpos = startpos
        self.endpos = endpos

        self.vertices = [startpos]
        self.edges = []
        self.success = False

        self.vex2idx = {startpos:0}
        self.neighbors = {0:[]}
        self.distances = {0:0.}

        self.sx = endpos[0] - startpos[0]
        self.sy = endpos[1] - startpos[1]

        #np.random.seed(0)
        #self.random_x = np.random.uniform(-40.0,40.0, 5001)
        #self.random_y = np.random.uniform(-40.0,80.0, 5001)

    def add_vex(self, pos):
        try:
            idx = self.vex2idx[pos]
        except:
            idx = len(self.vertices)
            self.vertices.append(pos)
            self.vex2idx[pos] = idx
            self.neighbors[idx] = []
        return idx

    def add_edge(self, idx1, idx2, cost):
        self.edges.append((idx1, idx2))
        self.neighbors[idx1].append((idx2, cost))
        self.neighbors[idx2].append((idx1, cost))


    def randomPosition(self):
        """ Sample random vertex. Constraints:
            - must be inside map (tackle that with a collision check instead "LineString")
            - must only cross lane when obstacle ahead"""


        #posx = self.random_x[idx]
        #posy = self.random_y[idx]
        posx = random.uniform(-40,40)
        posy = random.uniform(-40,80)
        return posx, posy

class RRT:
    """ Class rrt samples a RRT tree from a given startpos until a node close enough to enpos is found.
        This tolerance is defined by "radius". 
        Obstacles are static obstacles from the lidar scan.
        Class is inspired by https://gist.github.com/Fnjn/58e5eaa27a3dc004c3526ea82a92de80 """
    def __init__(
        self,
        startpos,
        endpos,
        obstacles,
        lanelet_network, 
        buffer
    ):
        self.startpos = startpos
        self.endpos = endpos
        self.obstacles = obstacles
        self.lanelet_network = lanelet_network
        # tune those parameters
        self.n_iter=5000 #5000
        self.stepSize=1.5 #1.7
        self.radius=2.5
        self.buffer = buffer
    
    def isThroughObstacle(self,nearvex,newvex):
        sSegment = LineString([(nearvex[0],nearvex[1]),(newvex[0], newvex[1])])
        for staticObstacle in self.obstacles:
            obstacle = staticObstacle.shape
            if obstacle.geom_type == "Polygon": # obstacle
                convex_hull = obstacle.convex_hull.buffer(2.5)
                # Create a larger polygon by buffering the original polygon by a distance of 0.5
                #larger_polygon = obstacle.buffer(self.buffer)
                if sSegment.intersects(convex_hull) is True:
                    return True
            if obstacle.geom_type == "LineString": # street boundary
                larger_polygon_from_lineString = sPolygon(obstacle).buffer(self.buffer)
                if sSegment.intersects(larger_polygon_from_lineString) is True:
                    return True
            if obstacle.geom_type == 'LinearRing': # street boundary
                larger_polygon_from_linearRing= sPolygon(obstacle).buffer(self.buffer)
                if sSegment.intersects(larger_polygon_from_linearRing) is True:
                    return True
        # no intersection detected
        return False
    
    def nearest(self,G,vex):
        """ Returns the nearest vertex v in G to randvex, if the edge v-randvex is collision free. """
        Nvex = None
        Nidx = None
        minDist = float("inf")

        for idx, v in enumerate(G.vertices):
            dist = np.linalg.norm(np.array(v)-np.array(vex)) # length of (v,vex)
            if dist < minDist:
                minDist = dist
                Nidx = idx
                Nvex = v

        return Nvex, Nidx

    def newVertex(self, randvex, nearvex):
        """ Selects a new vertex "newvex" by moving an incremental distance "stepSize" from nearvex in the direction of randvex """
        dirn = np.array(randvex) - np.array(nearvex)
        length = np.linalg.norm(dirn)
        dirn = (dirn / length) * min (self.stepSize, length)

        newvex = (nearvex[0]+dirn[0], nearvex[1]+dirn[1])
        return newvex

    def generate_RRT(self):
        """ RRT Algorithm """
        # Initialize graph with start and end position
        G = Graph(self.startpos, self.endpos)

        for _ in range(self.n_iter):
            # Sample random vertex "randvex"
            randvex = G.randomPosition()
            # Find the vertex in G (nearvex) that is collision free and nearest to the randomly sampled vertex (randvex)
            nearvex, nearidx = self.nearest(G, randvex)
            # Select new vertex
            newvex = self.newVertex(randvex, nearvex)

            # Add edge newvex-nearvex to the graph G if edge newvex-nearvex is collision free
            if self.isThroughObstacle(nearvex,newvex) is True:
                continue
            # check if new edge is outside lane network
            lanelet_ids = self.lanelet_network.find_lanelet_by_position([[newvex[0], newvex[1]], ])[0]
            if len(lanelet_ids) == 0:
                # newvex is outside lanelet
                continue
            newidx = G.add_vex(newvex)
            dist = np.linalg.norm(np.array(newvex)-np.array(nearvex))
            G.add_edge(newidx, nearidx, dist)

            dist = np.linalg.norm(np.array(newvex)-np.array(G.endpos))
            if dist < 2 * self.radius:
                #print(f"Goal reached in iteration {_}")
                endidx = G.add_vex(G.endpos)
                G.add_edge(newidx, endidx, dist)
                G.success = True
                #print("success", G.success)
                break # with break faster, without its possible to find a shorter path
        return G, G.success

    def dijkstra(self, G):
        """Dijkstra algorithm for finding shortest path from start position to end. Source https://gist.github.com/Fnjn/58e5eaa27a3dc004c3526ea82a92de80. """
        srcIdx = G.vex2idx[G.startpos]
        dstIdx = G.vex2idx[G.endpos] # access only succesful if goal was found by RRT

        # build dijkstra
        nodes = list(G.neighbors.keys())
        dist = {node: float('inf') for node in nodes}
        prev = {node: None for node in nodes}
        dist[srcIdx] = 0

        while nodes:
            curNode = min(nodes, key=lambda node: dist[node])
            nodes.remove(curNode)
            if dist[curNode] == float('inf'):
                break

            for neighbor, cost in G.neighbors[curNode]:
                newCost = dist[curNode] + cost
                if newCost < dist[neighbor]:
                    dist[neighbor] = newCost
                    prev[neighbor] = curNode

        # retrieve path
        path = deque()
        curNode = dstIdx
        while prev[curNode] is not None:
            path.appendleft(G.vertices[curNode])
            curNode = prev[curNode]
        path.appendleft(G.vertices[curNode])
        return list(path)

    def smooth_path(self, path, weight_data=0.8, weight_smooth=0.45, tol=0.001): # 0.00001
        """ https://gist.github.com/jaems33/5c61c288d0470d7ff5ed1ee9a00a7a3f#file-path_smoothing-py """
        new_path = np.copy(path)
        change = tol
        while change >= tol:
            change = 0.0
            for i in range(1, len(path)-1):
                for j in range(len(path[i])):
                    aux = new_path[i][j]
                    new_path[i][j] += weight_data * (path[i][j] - new_path[i][j])
                    new_path[i][j] += weight_smooth * (new_path[i-1][j] + new_path[i+1][j] - (2.0 * new_path[i][j]))
                    change += abs(aux - new_path[i][j])
        return new_path


    def plot(self, G, path=None):
        for staticObstacle in self.obstacles:
            obstacle = staticObstacle.shape
            if obstacle.geom_type == "Polygon":
                plt.plot(*obstacle.exterior.xy)
            else:
                plt.plot(*obstacle.xy)
        path_x, path_y = [],[]
        for coord in path:
            path_x.append(coord[0])
            path_y.append(coord[1])
        plt.scatter(path_x,path_y, s=0.5)
        plt.savefig("path.png", dpi=300)