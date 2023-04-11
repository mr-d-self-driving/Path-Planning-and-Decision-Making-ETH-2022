from abc import ABC, abstractmethod
from dataclasses import dataclass
from tkinter import E
from xxlimited import new
from osmnx.distance import great_circle_vec

from pdm4ar.exercises.ex02.structures import X, Path
from pdm4ar.exercises.ex03.structures import WeightedGraph, TravelSpeed


@dataclass
class InformedGraphSearch(ABC):
    graph: WeightedGraph

    @abstractmethod
    def path(self, start: X, goal: X) -> Path:
        # need to introduce weights!
        pass

@dataclass
class UniformCostSearch(InformedGraphSearch):
    def path(self, start: X, goal: X) -> Path:
        # todo
        
        # initialize Queue, CostToReach, Path and Parent
        Q = [start]
        CTR = {start: 0}
        P = []
        parent = {start:0}

        # traverse Graph with UCS
        while Q != []:
            s = Q.pop(0)
            if s == goal:
                # reconstruct path from goal to start
                while s != start:
                    P = P + [parent[s]]
                    s = parent[s]
                P = [goal] + P
                P = list(reversed(P))
                return P
            else:
                for node,edges in self.graph.adj_list.items():
                    if node == s:
                        edges = list(edges)
                        for i in range(len(edges)):
                            #new CTR = cost of parent + new cost
                            newCTR = CTR[s] + self.graph.get_weight(s, edges[i])
                            #CTR for unvisited nodes is +inf
                            if edges[i] not in CTR:
                                CTR[edges[i]] = float('inf')
                            #for nodes that have been visited: if newCTR of s' is smaller than its cost before
                            if newCTR < CTR[edges[i]]:
                                CTR[edges[i]] = newCTR
                                parent[edges[i]] = s
                                # add s' to the Queue and sort after priority (lowest cost)
                                Q_dict = {}
                                Q = Q + [edges[i]]
                                for node in Q:
                                        Q_dict[node] = CTR[node]
                                Q_dict = dict(sorted(Q_dict.items(), key=lambda item: item[1]))
                                Q = list(Q_dict.keys())

        return P

@dataclass
class Astar(InformedGraphSearch):

    def heuristic(self, u: X, v: X) -> float:
        # todo
        # check if heuristic is admissiable:
        # get distance btw two nodes
        # pairwise distance(u,v)
        #u_lat, u_long = self.graph.get_node_coordinates(u)
        #v_lat, v_long = self.graph.get_node_coordinates(v)
        #dist = great_circle_vec(u_lat,u_long,v_lat,v_long)
        #print(dist)
        #time_metric = dist/1000/TravelSpeed.HIGHWAY.value
        ## t_highway has to be faster than weight -> smaller
        return 0


        # calculate time = distance/speed for each pair to check how large the heuristic (distance) has to be to never be smaller (faster) than the weights (time)

    def path(self, start: X, goal: X) -> Path:
        # todo
        # initialize Queue, CostToReach, Path and Parent
        Q = [start]
        CTR = {start: 0}
        P = []
        parent = {start:0}

        # traverse Graph with UCS
        while Q != []:
            s = Q.pop(0)
            if s == goal:
                # reconstruct path from goal to start
                while s != start:
                    P = P + [parent[s]]
                    s = parent[s]
                P = [goal] + P
                P = list(reversed(P))
                return P
            else:
                for node,edges in self.graph.adj_list.items():
                    if node == s:
                        edges = list(edges)
                        for i in range(len(edges)):
                            #new CTR = h(s') cost of parent + new cost
                            newCTR = self.heuristic(edges[i],goal) + CTR[s] + self.graph.get_weight(s, edges[i])
                            #CTR for unvisited nodes is +inf
                            if edges[i] not in CTR:
                                CTR[edges[i]] = float('inf')
                            #for nodes that have been visited: if newCTR of s' is smaller than its cost before
                            if newCTR < CTR[edges[i]]:
                                CTR[edges[i]] = newCTR
                                parent[edges[i]] = s
                                # add s' to the Queue and sort after priority (lowest cost)
                                Q_dict = {}
                                Q = Q + [edges[i]]
                                for node in Q:
                                        Q_dict[node] = CTR[node]
                                Q_dict = dict(sorted(Q_dict.items(), key=lambda item: item[1]))
                                Q = list(Q_dict.keys())

        return P


def compute_path_cost(wG: WeightedGraph, path: Path):
    """A utility function to compute the cumulative cost along a path"""
    if not path:
        return float("inf")
    total: float = 0
    for i in range(1, len(path)):
        inc = wG.get_weight(path[i - 1], path[i])
        total += inc
    return total


###
