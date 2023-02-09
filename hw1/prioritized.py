import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost
from pdb import set_trace as bp

class PrioritizedPlanningSolver(object):
    """A planner that plans for each robot sequentially."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.CPU_time = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        start_time = timer.time()
        result = []
        constraints = []
        
        # Sample vertex constraint
        # constraints.append({'agent': 0,
        #                     'loc': [(1,5)],
        #                     'timestep': 4})

        # # Sample edge constraint
        # constraints.append({'agent': 1,
        #                     'loc': [(1,2),(1,3)],
        #                     'timestep': 1})

        max_path_len = sum([x.count(False) for x in self.my_map])

        for i in range(self.num_of_agents):  # Find path for each agent
            print("Num constrains {}".format(len(constraints)))
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i], i, constraints)
            if path is None:
                raise BaseException('No solutions')
            result.append(path)

            ##############################
            # Task 2.1/2.2: Add constraints here
            #         Useful variables:
            #            * path contains the solution path of the current (i'th) agent, e.g., [(1,1),(1,2),(1,3)]
            #            * self.num_of_agents has the number of total agents
            #            * constraints: array of constraints to consider for future A* searches

            ##############################

            for low_p_agent in range(i+1, self.num_of_agents):
                for path_timestep in range(len(path)):
                    # Adding vertex constraints
                    constraints.append({'agent': low_p_agent,
                        'loc': [path[path_timestep]],
                        'timestep': path_timestep})
                
                    # Adding edge constraint
                    if(path_timestep>0):
                        constraints.append({'agent': low_p_agent,
                            'loc': [path[path_timestep], path[path_timestep-1]],
                            'timestep': path_timestep})
                
                # Additional constraints for later timesteps
                for path_timestep in range(len(path),(max_path_len+1)):
                    # Adding vertex constraints
                    # bp()
                    constraints.append({'agent': low_p_agent,
                        'loc': [path[-1]],
                        'timestep': path_timestep})
                
        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        print(result)
        return result
