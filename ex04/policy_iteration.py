from typing import Tuple

import numpy as np

from pdm4ar.exercises.ex04.mdp import GridMdp, GridMdpSolver
from pdm4ar.exercises.ex04.structures import ValueFunc, Policy
from pdm4ar.exercises_def.ex04.utils import time_function
from functools import lru_cache


class PolicyIteration(GridMdpSolver):
    @staticmethod
    @time_function
    @lru_cache(maxsize=None)
    def solve(grid_mdp: GridMdp) -> Tuple[ValueFunc, Policy]:
        V = np.zeros_like(grid_mdp.grid).astype(float)
        policy = np.zeros_like(grid_mdp.grid).astype(int)
        
        # todo implement here
        gamma = grid_mdp.gamma
        dimx = grid_mdp.grid.shape[0]
        dimy = grid_mdp.grid.shape[1]
        x_start,y_start = grid_mdp.find_start()

        for i in range(90):
            # 1. Modified policy evaluation
            oldV = V.copy()
            for x in range(dimx):
                for y in range(dimy):
                    Q = {}
                    a = policy[(x,y)]

                    # edge
                    if x+1 == dimx:
                        #corner (x,y) and (x,0)
                        if y+1 == dimy or y+1 < 0:
                            t0 = 0.5/3 #offmap
                            r0 = grid_mdp.stage_reward((x_start,y_start))
                            oldV0 = oldV[(x_start,y_start)]
                        else:
                            #edge
                            t0 = 0.25/3 #offmap
                            r0 = grid_mdp.stage_reward((x_start,y_start))
                            oldV0 = oldV[(x_start,y_start)]
                    else:
                        t0 = grid_mdp.get_transition_prob((x,y),a,(x+1,y)) #south
                        r0 = grid_mdp.stage_reward((x,y))
                        oldV0 = oldV[(x+1,y)]

                    #edge
                    if x-1 < 0:
                        #corner (0,y) and (0,0)
                        if y+1 == dimy or y+1 < 0:
                            t1 = 0.5/3.0
                            r1 = grid_mdp.stage_reward((x_start,y_start))
                            oldV1 = oldV[(x_start,y_start)]
                        else:
                            t1 = 0.25/3.0
                            r1 = grid_mdp.stage_reward((x_start,y_start))
                            oldV1 = oldV[(x_start,y_start)]
                    else:
                        t1 = grid_mdp.get_transition_prob((x,y),a,(x-1,y)) #north
                        r1 = grid_mdp.stage_reward((x,y))
                        oldV1 = oldV[(x-1,y)]

                    if y+1 == dimy:
                        t2 = 0.25/3.0
                        r2 = grid_mdp.stage_reward((x_start,y_start))
                        oldV2 = oldV[(x_start,y_start)]
                    else:
                        t2 = grid_mdp.get_transition_prob((x,y),a,(x,y+1)) #east
                        r2 = grid_mdp.stage_reward((x,y))
                        oldV2 = oldV[(x,y+1)]

                    if y-1 <0:
                        t3 = 0.25/3.0
                        r3= grid_mdp.stage_reward((x_start,y_start))
                        oldV3 = oldV[(x_start,y_start)]
                    else:
                        t3 = grid_mdp.get_transition_prob((x,y),a,(x,y-1)) #west
                        r3= grid_mdp.stage_reward((x,y))
                        oldV3 = oldV[(x,y-1)]

                    t4 = grid_mdp.get_transition_prob((x,y),a,(x,y)) #stay
                    r4 = grid_mdp.stage_reward((x,y))
                    oldV4 = oldV[(x,y)]

                    SUM0 = t0*(r0+gamma*oldV0)
                    SUM1 = t1*(r1+gamma*oldV1)
                    SUM2 = t2*(r2+gamma*oldV2)
                    SUM3 = t3*(r3+gamma*oldV3)
                    SUM4 = t4*(r4+gamma*oldV4)
                    Q[a] = SUM0 + SUM1 + SUM2 + SUM3 + SUM4
                    V[(x,y)] = max(Q.values())

            # 2. Policy iteration
            for x in range(dimx):
                for y in range(dimy):
                    Q = {}
                    actions = grid_mdp.valid_action((x,y))
                    for a in actions:
                        # sum over all s'
                        if x+1 == dimx:
                            t0 = 0.25/3 #offmap
                            r0 = grid_mdp.stage_reward((x_start,y_start))
                            V0 = V[(x_start,y_start)]
                        else:
                            t0 = grid_mdp.get_transition_prob((x,y),a,(x+1,y)) #south
                            r0 = grid_mdp.stage_reward((x,y))
                            V0 = V[(x+1,y)]

                        if x-1 < 0:
                            t1 = 0.25/3.0
                            r1 = grid_mdp.stage_reward((x_start,y_start))
                            V1 = V[(x_start,y_start)]
                        else:
                            t1 = grid_mdp.get_transition_prob((x,y),a,(x-1,y)) #north
                            r1 = grid_mdp.stage_reward((x,y))
                            V1 = V[(x-1,y)]

                        if y+1 == dimy:
                            t2 = 0.25/3.0
                            r2 = grid_mdp.stage_reward((x_start,y_start))
                            V2 = V[(x_start,y_start)]
                        else:
                            t2 = grid_mdp.get_transition_prob((x,y),a,(x,y+1)) #east
                            r2 = grid_mdp.stage_reward((x,y))
                            V2 = V[(x,y+1)]

                        if y-1 <0:
                            t3 = 0.25/3.0
                            r3= grid_mdp.stage_reward((x_start,y_start))
                            V3 = V[(x_start,y_start)]
                        else:
                            t3 = grid_mdp.get_transition_prob((x,y),a,(x,y-1)) #west
                            r3= grid_mdp.stage_reward((x,y))
                            V3 = V[(x,y-1)]

                        t4 = grid_mdp.get_transition_prob((x,y),a,(x,y)) #stay
                        r4 = grid_mdp.stage_reward((x,y))
                        V4 = V[(x,y)]

                        SUM0 = t0*(r0+gamma*V0)
                        SUM1 = t1*(r1+gamma*V1)
                        SUM2 = t2*(r2+gamma*V2)
                        SUM3 = t3*(r3+gamma*V3)
                        SUM4 = t4*(r4+gamma*V4)
                        Q[a] = SUM0 + SUM1 + SUM2 + SUM3 + SUM4
                        # update policy
                        policy[(x,y)] = max(Q,key=Q.get)
            

        return V, policy
