import time as timer
import heapq
from collections import deque
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost
from topological_sort import TopologyGraph
from cbs import detect_collisions 

def generate_priority_pairs(collision):
    priority_pairs = []
    ##############################
    # Task 4.1: Generate a priority pair from a collision

    ##############################

    return priority_pairs

def get_lower_priority_agents(priority_pairs, agent):
    tg = TopologyGraph(directed=True)
    tg.clear_graph()

    # construct graph
    for pair in priority_pairs:
        tg.Edge(pair[0], pair[1])

    if not tg.has_node(agent):
        return [agent]

    # Get the nodes behind a given node in a topological ordering
    return tg.get_subsequent_nodes_in_topological_ordering(agent)

def get_higher_priority_agents(priority_pairs, agent):
    tg = TopologyGraph(directed=True)
    tg.clear_graph()

    # construct graph
    for pair in priority_pairs:
        tg.Edge(pair[1], pair[0])

    if not tg.has_node(agent):
        return [agent]

    # Get the nodes behind a given node in a topological ordering
    return tg.get_subsequent_nodes_in_topological_ordering(agent)

def collide_with_higher_priority_agents(node, agent):
        collisions = node['collisions']
        priority_pairs = node['priority_pairs']

        if collisions == [] or priority_pairs == []:
            return []

        higher_priority_agents = get_higher_priority_agents(node['priority_pairs'], agent)

        for collision in collisions:
            if collision['a1'] == agent and collision['a2'] in higher_priority_agents:
                return True
            elif collision['a2'] == agent and collision['a1'] in higher_priority_agents:
                return True

        return False 

class PBSSolver(object):
    """The high-level search of PBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []
        self.search_stack = deque()

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        # Max heap
        heapq.heappush(self.open_list, (-node['cost'], -len(node['collisions']), self.num_of_generated, node))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        self.num_of_expanded += 1
        return node
            
    def update_plan(self, node, i):
        ##############################
        # Task 4.2: High-Level Search
        
        ##############################
        
        return True


    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations
        """

        print('Start PBS')
        self.start_time = timer.time()

        # Generate the root node
        # priority_pairs   - list of priority pairs
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions     - list of collisions in paths
        root = {'cost': 0,
                'priority_pairs': [],
                'paths': [],
                'collisions': []}
        for i in range(self.num_of_agents):  # Find initial path for each agent
            self.update_plan(root,i)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.search_stack.append(root)

        ##############################
        # Task 4.2: High-Level Search
        #           Repeat the following as long as the search_stack is not empty:
        #             1. Get the next node from the search_stack (you can use self.search_stack.pop()
        #             2. If this node has no collision, return solution
        #             3. Otherwise, choose the first collision and generate a priority pair (using your
        #                generate_priority_pairs function). Add a new child node to your search_stack for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit
        
        ##############################
        raise BaseException('No solutions')


    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))



if __name__ == '__main__':
    solver = PBSSolver([], [], [])

    node1 = {'cost': 300,
            'priority_pairs': [],
            'paths': [],
            'collisions': []}
    
    node2 = {'cost': 200,
            'priority_pairs': [],
            'paths': [],
            'collisions': []}
    
    node3 = {'cost': 100,
            'priority_pairs': [],
            'paths': [],
            'collisions': []}

    solver.push_node(node1)
    solver.push_node(node2)
    solver.push_node(node3)

    print(solver.pop_node())
    print(solver.pop_node())
    print(solver.pop_node())
