import numpy as np
from pulp import *

from pdm4ar.exercises_def.ex07.structures import (
    ProblemVoyage,
    OptimizationCost,
    Island,
    Constraints,
    Feasibility,
    SolutionVoyage,
)


def solve_optimization(problem: ProblemVoyage) -> SolutionVoyage:
    """
    Solve the optimization problem enforcing the requested constraints.
    Parameters
    ---
    problem : ProblemVoyage
        Contains the problem data: cost to optimize, starting crew, tuple of islands,
        and information about the requested constraint (the constraints not set to `None` +
        the `voyage_order` constraint)
    Returns
    ---
    out : SolutionVoyage
        Contains the feasibility status of the problem, and the optimal voyage plan
        as a list of ints if problem is feasible, else `None`.
    """
    
    islands = problem.islands
    constraints = problem.constraints

    """ Constants """
    # number of total archipelagos
    N_total = (islands[len(islands)-1].arch) + 1
    # number of archipelagos without start and goal arch
    N = N_total -2
    # number of total islands
    I_total = len(islands)
    # number of islands without first and last island
    I = I_total -2
    # number of islands per archipelago
    k = int(I/N)
    
    """ Decision variables """
    vars = LpVariable.dicts("Island", np.arange(I_total), 0, 1, LpInteger) # Binary decision variables represent if island i is visited
    vars_c = LpVariable.dicts("y",np.arange(I_total),0,cat=LpContinuous)   # Continous decision variables for implementing the min max cost function
    vars_L1_x = LpVariable.dicts("L1_x",np.arange(N_total),0,cat=LpContinuous) # Continous decision variables for the absolute values of delta x
    vars_L1_y = LpVariable.dicts("L1_y",np.arange(N_total),0,cat=LpContinuous) # Continous decision variables for the absolute values of delta y

    """ Costs """
    if problem.optimization_cost == OptimizationCost.min_total_nights:
        prob = LpProblem("Minimize_nights_to_complete_the_voyage", LpMinimize)
        prob += lpSum([vars[i]*islands[i].nights for i in range(I_total)]), "Sum_of_minimum_nights_spent_on_island",

    if problem.optimization_cost == OptimizationCost.max_final_crew:
        prob = LpProblem("Maximize_final_crew_size", LpMaximize)
        prob += problem.start_crew*vars[0] + lpSum([vars[i]*islands[i].delta_crew for i in range(1,I_total)]), "Sum_of_crew_size"

    if problem.optimization_cost == OptimizationCost.min_total_sailing_time:
        prob = LpProblem("Minimize_total_sailing_time", LpMinimize)
        prob += lpSum([vars[i]*(islands[i].arrival - islands[i].departure) for i in range(1,I_total-1)]) + (vars[I_total-1]*islands[I_total-1].arrival-vars[0]* islands[0].departure), "Sum_of_sailing_time"

    if problem.optimization_cost == OptimizationCost.min_total_travelled_L1_distance:
        prob = LpProblem("Minimize_total_L1_norm", LpMinimize)
        prob += lpSum((vars_L1_x[a] + vars_L1_y[a]) for a in range(N_total)), "Sum_of_L1_distance_of_journeys_btw_arch=a+1_and_arch=a"
        # add extra constraints
        for a in range(N_total-1):
            # contains diffs in x between all islands on archipel a and a+1 in one iteration
            delta_x = (lpSum([vars[i]*islands[i].x for i in range(I_total) if islands[i].arch == a+1]) - lpSum([vars[i]*islands[i].x for i in range(I_total) if islands[i].arch == a])) 
            # contains diffs in y between all islands on archipel a and a+1 in one iteration
            delta_y = (lpSum([vars[i]*islands[i].y for i in range(I_total) if islands[i].arch == a+1]) - lpSum([vars[i]*islands[i].y for i in range(I_total) if islands[i].arch == a]))   
            # vars_L1_x[a] = abs of x_a+1 - x_a
            prob += vars_L1_x[a] >= delta_x, (f"L1_1_x{a}")
            prob += vars_L1_x[a] >= -delta_x, (f"L1_2_x{a}")
            # vars_L1_y[a] = abs of y_a+1 - y_a
            prob += vars_L1_y[a] >= delta_y, (f"L1_3_y{a}")
            prob += vars_L1_y[a] >= -delta_y, (f"L1_4_y{a}")

    if problem.optimization_cost == OptimizationCost.min_max_sailing_time:
        prob = LpProblem("Minimize_maximum_individual_sailing_time", LpMinimize)
        prob += lpSum(vars_c), "Sum_of_continous_decision_vars"
        # add extra constraints
        for a in range(N_total-1):
            prob += lpSum([-vars[i]*islands[i].departure for i in range(I_total) if islands[i].arch == a]) + lpSum([vars[j]*islands[j].arrival for j in range(I_total) if islands[j].arch == a+1]) <= vars_c, (f"Min_max_{a,a+1}")


    """ Constraints """
    # Voyage constraint
    for a in range(N_total):
        prob+= lpSum([1*vars[i] if islands[i].arch == a else 0*vars[i] for i in range(I_total)]) ==1, (f"Archipelago_{a}"),
    
    # Minimum nights constraint
    if constraints.min_nights_individual_island is not None:
        for a in range(1,N_total-1):
            prob+= lpSum([islands[i].nights*vars[i] if islands[i].arch == a else 0*vars[i] for i in range(1,I_total-1)]) >= constraints.min_nights_individual_island, (f"Minimum_nights_per_island_on_arch_{a}")
    
    # Minimum crew size constraint
    if constraints.min_total_crew is not None:
        prob += problem.start_crew*vars[0] >= constraints.min_total_crew, ("Min_start_crew")
        for j in range(1,I_total):
            prob+= problem.start_crew*vars[0] + lpSum([(islands[i].delta_crew * vars[i]) for i in range(1,j)]) >= constraints.min_total_crew, (f'Minimum_total_crew_island_{j}')
        
    # Maximum crew size constraint
    if constraints.max_total_crew is not None:
        prob += problem.start_crew*vars[0] <= constraints.max_total_crew, ("Max_start_crew")
        for j in range(1,I_total):
            prob+= problem.start_crew*vars[0] + lpSum([(islands[i].delta_crew * vars[i]) for i in range(1,j)]) <= constraints.max_total_crew, (f'Maximum_total_crew_island_{j}')
    
    # Maximum duration individual journey constraint
    if constraints.max_duration_individual_journey is not None:
        for a in range(N_total-1):
            prob += lpSum([-vars[i]*islands[i].departure for i in range(I_total) if islands[i].arch == a]) + lpSum([vars[j]*islands[j].arrival for j in range(I_total) if islands[j].arch == a+1]) <= constraints.max_duration_individual_journey, (f'Maximum_duration_{a,a+1}')
    
    # Maximum L1-norm distance individual journey constraint
    if constraints.max_L1_distance_individual_journey is not None:
        for a in range(N_total-1):
            prob += (lpSum([vars[i]*islands[i].x for i in range(I_total) if islands[i].arch == a]) - lpSum([vars[i]*islands[i].x for i in range(I_total) if islands[i].arch == a+1])) + (lpSum([vars[i]*islands[i].y for i in range(I_total) if islands[i].arch == a]) - lpSum([vars[i]*islands[i].y for i in range(I_total) if islands[i].arch == a+1])) <= constraints.max_L1_distance_individual_journey, (f"L1_1{(a,a+1)}")
            prob += (lpSum([vars[i]*islands[i].x for i in range(I_total) if islands[i].arch == a+1]) - lpSum([vars[i]*islands[i].x for i in range(I_total) if islands[i].arch == a])) + (lpSum([vars[i]*islands[i].y for i in range(I_total) if islands[i].arch == a+1]) - lpSum([vars[i]*islands[i].y for i in range(I_total) if islands[i].arch == a])) <= constraints.max_L1_distance_individual_journey, (f"L1_2{(a,a+1)}")
            prob += (lpSum([vars[i]*islands[i].x for i in range(I_total) if islands[i].arch == a]) - lpSum([vars[i]*islands[i].x for i in range(I_total) if islands[i].arch == a+1])) + (lpSum([vars[i]*islands[i].y for i in range(I_total) if islands[i].arch == a+1]) - lpSum([vars[i]*islands[i].y for i in range(I_total) if islands[i].arch == a])) <= constraints.max_L1_distance_individual_journey, (f"L1_3{(a,a+1)}")
            prob += (lpSum([vars[i]*islands[i].x for i in range(I_total) if islands[i].arch == a+1]) - lpSum([vars[i]*islands[i].x for i in range(I_total) if islands[i].arch == a])) + (lpSum([vars[i]*islands[i].y for i in range(I_total) if islands[i].arch == a]) - lpSum([vars[i]*islands[i].y for i in range(I_total) if islands[i].arch == a+1])) <= constraints.max_L1_distance_individual_journey, (f"L1_4{(a,a+1)}")
    
    """ Solve the mixed linear integer program """
    # The problem data is written to an .lp file and solved in the next step
    prob.writeLP("Pirate_ship.lp")
    prob.solve()

    # Get problem status and value of decision variables
    if LpStatus[prob.status] == 'Optimal':
        feasibility = Feasibility.feasible
        voyage_plan = [i for i in sorted(vars.keys()) if vars[i].varValue == 1]
    else:
        feasibility = Feasibility.unfeasible
        voyage_plan = [] 
    return SolutionVoyage(feasibility, sorted(voyage_plan))