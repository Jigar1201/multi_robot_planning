import time as timer
from single_agent_planner import compute_heuristics, get_sum_of_cost, push_node, pop_node, all_in_map, get_path, is_valid_motion, compare_nodes 
import heapq
import itertools

class JointStateSolver(object):
    """A planner that plans for all robots together."""

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
    
    def move_joint_state(self, locs, dir):

        new_locs = []
        for i in range(len(locs)):
            new_locs.append((locs[i][0] + dir[i][0], locs[i][1] + dir[i][1]))
        return new_locs

    def generate_motions_recursive(self, num_agents, cur_agent):
        directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0, 0)]
        
        joint_state_motions = []

        if cur_agent == num_agents:
            return joint_state_motions

        for dir in directions:
            next_motion = [dir]
            motions = self.generate_motions_recursive(num_agents, cur_agent+1)
            if len(motions) == 0:
                joint_state_motions.append(next_motion)
            else:
                for motion in motions:
                    joint_state_motions.append(next_motion + motion)
        return joint_state_motions

    def check_is_valid(self, curr_loc, new_loc, my_map):
        
        # Checking for collision with environment
        for agent_id in range(len(new_loc)):
            agent_loc = new_loc[agent_id]

            # Check for bounds for new location
            if(agent_loc[0]<0 or agent_loc[0]>=len(my_map)):
                return False
            if(agent_loc[1]<0 or agent_loc[1]>=len(my_map[0])):
                return False
            if(my_map[agent_loc[0]][agent_loc[1]]):
                # print("Agent : ",agent_id," has env collision")
                return False
            for agent_id_2 in range(len(new_loc)):
                if(agent_id_2==agent_id):
                    continue
                # Checking for collision amongst new locations(also vertex collision)
                if(new_loc[agent_id_2]==new_loc[agent_id]):
                    # print("Agents : ",agent_id," and ",agent_id_2,"has vertex collision")
                    return False
                # Checking for edge collision
                if(curr_loc[agent_id_2]==new_loc[agent_id] and new_loc[agent_id_2]==curr_loc[agent_id]):
                    # print("Agents : ",agent_id," and ",agent_id_2,"has edge collision")
                    return False
        return True


    def joint_state_a_star(self, my_map, starts, goals, h_values, num_agents):
        """ my_map      - binary obstacle map
            start_loc   - start position
            goal_loc    - goal position
            agent       - the agent that is being re-planned
            constraints - constraints defining where robot should or cannot go at each timestep
        """
    
        ##############################
        # Task 1.1: Extend the A* search to search in the joint configurations of agents.
        open_list = []
        closed_list = dict()

        h_value = 0
        for agent in range(num_agents):
            h_value += h_values[agent][starts[agent]]

        root = {'loc': starts, 'g_val': 0, 'h_val': h_value, 'parent': None}
        closed_list[tuple(root['loc'])] = root
        heapq.heappush(open_list, (root['g_val'] + root['h_val'], root['h_val'], root['loc'], root))
        print("Len(open_list)  :",len(open_list))

        while len(open_list)>0:
            print("-------------------")
            _,_,_, curr_node = heapq.heappop(open_list)
            print("curr_node : ",curr_node)
            # Found solution
            if curr_node['loc'] == goals:
                print("Found the solution")
                return get_path(curr_node)

            joint_state_motions = self.generate_motions_recursive(num_agents, 0)
            for joint_state_motion in joint_state_motions:
                # Generate possible new locations
                new_locations = self.move_joint_state(curr_node['loc'], joint_state_motion)
                if(curr_node['loc'] == new_locations):
                    continue
                if not(self.check_is_valid(curr_node['loc'], new_locations, my_map)):
                    # print("Found invalid locations : ",new_locations)
                    continue    
                # print("Found valid locations : ",new_locations)
                child_h_value = 0
                for agent in range(num_agents):
                    child_h_value += h_values[agent][new_locations[agent]]
                child_node = {'loc': new_locations, 'g_val': curr_node['g_val'] + num_agents, 'h_val': child_h_value, 'parent': curr_node}
                
                # Check if this node already exists in open list
                found = False
                for i in range(len(open_list)):
                    if(open_list[i][2] == child_node['loc']):
                        found = True
                        # print("Found exisiting node")
                        if((child_node['g_val'] + child_node['h_val']) < open_list[i][0]):
                            open_list[i] = (child_node['g_val'] + child_node['h_val'], child_node['h_val'], child_node['loc'], child_node)
                if not found:
                    if(tuple(child_node['loc']) in closed_list):
                        existing_node = closed_list[tuple(child_node['loc'])]
                        if((child_node['g_val'] + child_node['h_val']) < 
                            (existing_node['g_val'] + existing_node['h_val'])):
                            closed_list[tuple(child_node['loc'])] = child_node
                            heapq.heappush(open_list, (child_node['g_val'] + child_node['h_val'], child_node['h_val'], child_node['loc'], child_node))
                    else:
                        closed_list[tuple(child_node['loc'])] = child_node
                        heapq.heappush(open_list, (child_node['g_val'] + child_node['h_val'], child_node['h_val'], child_node['loc'], child_node))

        ##############################

        return None  # Failed to find solutions

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        start_time = timer.time()
        result = []

        path = self.joint_state_a_star(self.my_map, self.starts, self.goals, self.heuristics,self.num_of_agents)

        if path is None:
            raise BaseException('No solutions')
                
        # Convert the path to a list of paths for each agent
        for i in range(self.num_of_agents):
            result.append([])
            for node in path:
                result[i].append(node[i])
        # Delete duplicate goal positions
        final_paths = []
        for path in result: 
            goal = None
            num_delete = 0
            for point in reversed(path):
                if goal == None:
                    goal = point
                elif point == goal:
                    num_delete = num_delete + 1
                else:
                    break 
            if num_delete > 0:
                path = path[:-num_delete]
            final_paths.append(path)

        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(final_paths)))

        return final_paths
