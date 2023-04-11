from abc import abstractmethod, ABC
from typing import Tuple

from pdm4ar.exercises.ex02.structures import AdjacencyList, X, Path, OpenedNodes


class GraphSearch(ABC):
    @abstractmethod
    def search(self, graph: AdjacencyList, start: X, goal: X) -> Tuple[Path, OpenedNodes]:
        """
        :param graph: The given graph as an adjacency list
        :param start: The initial state (i.e. a node)
        :param goal: The goal state (i.e. a node)
        :return: The path from start to goal as a Sequence of states, None if a path does not exist
        """
        pass


class DepthFirst(GraphSearch):
    def search(self, graph: AdjacencyList, start: X, goal: X) -> Tuple[Path, OpenedNodes]:
        # todo implement here your solution

        # initialize queue, visited set, path, parent set
        Q = [start]
        Qcopy = []
        V = set()
        V.add(start)
        P = []
        parent = {}

        # traverse Graph with DFS
        while Q != []:
            s = Q.pop(0)
            Qcopy = Qcopy + [s]
            if s == goal:
                # reconstruct the path from goal to start
                while s != start:
                    P = P + [parent[s]]
                    s = parent[s]
                P = [goal] + P
                P = list(reversed(P))
                return P, Qcopy
            else:
                for node,edges in graph.items():
                    if node == s:
                        edges = list(edges)
                        #sort descending
                        edges.sort(reverse=True)
                        for i in range(len(edges)):
                            if edges[i] not in V:
                                if edges[i] not in Q:
                                    # add adjacent nodes to the front of the queue
                                    Q = [edges[i]] + Q
                                    V.add(edges[i])
                                parent[edges[i]] = s                          
        return P, Qcopy


class BreadthFirst(GraphSearch):
    def search(self, graph: AdjacencyList, start: X, goal: X) -> Tuple[Path, OpenedNodes]:
        # todo implement here your solution

        # initialize queue, visited set, path, parent set
        Q = [start]
        Qcopy = []
        V = set()
        V.add(start)
        P = []
        parent = {}

        # traverse Graph with BFS
        while Q != []:
            s = Q.pop(0)
            Qcopy = Qcopy + [s]
            if s == goal:
                # reconstruct path from goal to start
                while s != start:
                    P = P + [parent[s]]
                    s = parent[s]
                P = [goal] + P
                P = list(reversed(P))
                return P, Qcopy
            else:
                for node,edges in graph.items():
                    if node == s:
                        edges = list(edges)
                        #sort ascending
                        edges.sort()
                        for i in range(len(edges)):
                            if edges[i] not in V:
                                if edges[i] not in Q:
                                    # BFS adds adjacent node to the end of the queue
                                    Q = Q + [edges[i]]
                                    V.add(edges[i])
                                parent[edges[i]] = s
                                    
        return P,Qcopy



class IterativeDeepening(GraphSearch):
    def search(self, graph: AdjacencyList, start: X, goal: X) -> Tuple[Path, OpenedNodes]:
        # todo implement here your solution

        Q = [start]
        Qcopy = []
        V = set()
        V.add(start)

        # traverse graph with BFS
        while Q != []:
            s = Q.pop(0)
            Qcopy = Qcopy + [s]
            for node,edges in graph.items():
                if node == s:
                    edges = list(edges)
                    #sort ascending
                    edges.sort()
                    for i in range(len(edges)):
                        if edges[i] not in V:
                            if edges[i] not in Q:
                                # BFS adds adjacent node to the end of the queue
                                Q = Q + [edges[i]]
                                V.add(edges[i])


            # smallGraph only consists of nodes found by BFS for each depth (in Qcopy)
            smallGraph = {}
            for node in Qcopy:
                for key,items in graph.items():
                    if node == key:
                        smallGraph[key] = items

            #run DFS for each depth
            dfs = DepthFirst()
            P,Qcopy = dfs.search(smallGraph,start,goal)
            #valid path was found
            if P != []:
                return P,Qcopy

        return P,Qcopy
