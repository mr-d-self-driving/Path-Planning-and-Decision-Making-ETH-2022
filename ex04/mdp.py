from abc import ABC, abstractmethod
from typing import Tuple

import numpy as np
from numpy.typing import NDArray
from pdm4ar.exercises.ex04.structures import Action, Policy, State, ValueFunc


class GridMdp:
    def __init__(self, grid: NDArray[np.int], gamma: float = 0.9):
        assert len(grid.shape) == 2, "Map is invalid"
        self.grid = grid
        """The map"""
        self.gamma: float = gamma
        """Discount factor"""

    def get_transition_prob(self, state: State, action: Action, next_state: State) -> float:
        """Returns P(next_state | state, action)"""
        # todo

        # goal
        if self.grid[state] == 0:
            if action == 4 and next_state == state: 
                return 1.0
            else:
                return 0.0
        else:
            return self.center_prob(state,action,next_state)


    def center_prob(self, state: State, action: Action, next_state: State) -> float:
        #GRASS cells and START cell
        #South chosen
        if self.grid[state] == 2 or self.grid[state] == 1:
            if action == 2 and next_state == (state[0]+1,state[1]): #P(next_state = (i+1,j) | (i,j), South)
                return 0.75
            if action == 2 and next_state == (state[0]-1, state[1]): #north
                return 0.25/3.0
            if action == 2 and next_state == (state[0], state[1]-1): #west
                return 0.25/3.0
            if action == 2 and next_state == (state[0], state[1]+1): #east
                return 0.25/3.0 
        
        #West chosen
        if self.grid[state] == 2 or self.grid[state] == 1:
            if action == 1 and next_state == (state[0]+1,state[1]): #south
                return 0.25/3.0
            if action == 1 and next_state == (state[0]-1, state[1]): #north
                return 0.25/3.0
            if action == 1 and next_state == (state[0], state[1]-1): #west
                return 0.75
            if action == 1 and next_state == (state[0], state[1]+1): #east
                return 0.25/3.0

        #North chosen
        if self.grid[state] == 2 or self.grid[state] == 1:
            if action == 0 and next_state == (state[0]+1,state[1]): #south
                return 0.25/3.0
            if action == 0 and next_state == (state[0]-1, state[1]): #north
                return 0.75
            if action == 0 and next_state == (state[0], state[1]-1): #west
                return 0.25/3.0
            if action == 0 and next_state == (state[0], state[1]+1): #east
                return 0.25/3.0
        
        #East chosen
            if action == 3 and next_state == (state[0]+1,state[1]): #south
                return 0.25/3.0
            if action == 3 and next_state == (state[0]-1, state[1]): #north
                return 0.25/3.0
            if action == 3 and next_state == (state[0], state[1]-1): #west
                return 0.25/3.0
            if action == 3 and next_state == (state[0], state[1]+1): #east
                return 0.75

        #SWAMP Cells
        #South chosen
        if self.grid[state] == 3:
            if action == 2 and next_state == state: # doesn't move regardless the action taken
                return 0.25
            if action == 2 and next_state == (state[0]+1,state[1]): #south
                return 0.5
            if action == 2 and next_state == (state[0]-1, state[1]): #north
                return 0.25/3.0
            if action == 2 and next_state == (state[0], state[1]-1): #west
                return 0.25/3.0
            if action == 2 and next_state == (state[0], state[1]+1): #east
                return 0.25/3.0

        #West chosen
        if self.grid[state] == 3:
            if action == 1 and next_state == state: # doesn't move regardless the action taken
                return 0.25
            if action == 1 and next_state == (state[0]+1,state[1]): #south
                return 0.25/3.0
            if action == 1 and next_state == (state[0]-1, state[1]): #north
                return 0.25/3.0
            if action == 1 and next_state == (state[0], state[1]-1): #west
                return 0.5
            if action == 1 and next_state == (state[0], state[1]+1): #east
                return 0.25/3.0

        #East chosen
        if self.grid[state] == 3:
            if action == 3 and next_state == state: # doesn't move regardless the action taken
                return 0.25
            if action == 3 and next_state == (state[0]+1,state[1]): #south
                return 0.25/3.0
            if action == 3 and next_state == (state[0]-1, state[1]): #north
                return 0.25/3.0
            if action == 3 and next_state == (state[0], state[1]-1): #west
                return 0.25/3.0
            if action == 3 and next_state == (state[0], state[1]+1): #east
                return 0.5
        
        #North chosen
        if self.grid[state] == 3:
            if action == 0 and next_state == state: # doesn't move regardless the action taken
                return 0.25
            if action == 0 and next_state == (state[0]+1,state[1]): #south
                return 0.25/3.0
            if action == 0 and next_state == (state[0]-1, state[1]): #north
                return 0.5
            if action == 0 and next_state == (state[0], state[1]-1): #west
                return 0.25/3.0
            if action == 0 and next_state == (state[0], state[1]+1): #east
                return 0.25/3.0
        return 0.0

    def stage_reward(self, state: State) -> float:
        # todo
        if self.grid[state] == 2 or self.grid[state] ==1:
            return -1.0
        if self.grid[state] == 3:
            return -2.0
        if self.grid[state] == 0:
            return 10.0

    def find_start(self):
        x,y = np.where(self.grid == 1)
        return x,y
        
    def valid_action(self, state):

        dimx = self.grid.shape[0]-1
        dimy = self.grid.shape[1]-1
        
        #goal
        if self.grid[state] == 0:
            return np.array([4])

        # corners
        if state == (0,0):
            return np.array([3,2])
        if state == (dimx,0):
            return np.array([0,3])
        if state == (dimx,dimy):
            return np.array([0,1])
        if state == (0,dimy):
            return np.array([1,2])

        for x in range(self.grid.shape[0]):
            for y in range(self.grid.shape[1]):
                #edges
                if state ==(0,y):
                    return np.array([1,2,3]) 
                if state ==(x,0):
                    return np.array([0,3,2])
                if state ==(dimx,y):
                    return np.array([1,0,3])
                if state ==(x,dimy):
                    return np.array([0,1,2]) 
        #center
        return np.array([0,1,2,3])


class GridMdpSolver(ABC):
    @staticmethod
    @abstractmethod
    def solve(grid_mdp: GridMdp) -> Tuple[ValueFunc, Policy]:
        pass
