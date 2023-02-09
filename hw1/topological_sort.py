from collections import defaultdict

'''
Note : Node ids should be integers in the range [1, n] where n is the number of nodes in the graph
'''
class TopologyGraph:
    def __init__(self, directed=False):
        self.graph = defaultdict(list)
        self.vertices = []
        self.directed = directed
    
    def has_node(self, node):
        return node in self.graph

    def Edge(self, frm, to):
        if frm not in self.vertices:
            self.vertices.append(frm)
        if to not in self.vertices:
            self.vertices.append(to)

        self.graph[frm].append(to)
        if self.directed is False:
            self.graph[to].append(frm)

    def visit(self, s, visited, sortlist):
        visited[s] = True

        for i in self.graph[s]:
            if not visited[i]:
                self.visit(i, visited, sortlist)

        sortlist.insert(0, s)
    
    def get_subsequent_nodes_in_topological_ordering(self, node):
        if(self.is_cyclic() == 1):
            return [] 
        
        visited = {i: False for i in self.graph}
        
        sortlist = []

        if not visited[node]:
            self.visit(node, visited, sortlist)

        return sortlist

    def is_cyclic_util(self, v, visited, recStack):

        # Mark current node as visited and
        # adds to recursion stack
        visited[v] = True
        recStack[v] = True

        # Recur for all neighbours
        # if any neighbour is visited and in
        # recStack then graph is cyclic
        for neighbour in self.graph[v]:
            if visited[neighbour] == False:
                if self.is_cyclic_util(neighbour, visited, recStack) == True:
                    return True
            elif recStack[neighbour] == True:
                return True

        # The node needs to be poped from
        # recursion stack before function ends
        recStack[v] = False
        return False

	# Returns true if graph is cyclic else false
    def is_cyclic(self):
        self.V = len(self.vertices) #total vertices

        visited = {i: False for i in self.vertices}
        recStack = {i: False for i in self.vertices}

        for node in list(self.graph):
            if visited[node] == False:
                if self.is_cyclic_util(node, visited, recStack) == True:
                    return True
        return False
    
    def clear_graph(self):
        self.graph.clear()
        self.vertices = []
        self.V=0


#driver code
if __name__ == '__main__':
 
    g = TopologyGraph(directed=True)

    g.Edge(1, 2)
    g.Edge(1, 3)
    g.Edge(3, 4)
    g.Edge(4, 5)
    g.Edge(5, 6)
    g.Edge(6, 7)
    g.Edge(8, 1)
    
    g.get_subsequent_nodes_in_topological_ordering(2)
    # if g.isCyclic():
    #     print("Graph has a cycle")
    g.clear_graph()

    g.Edge(1, 2)
    g.Edge(1, 3)
    
    g.get_subsequent_nodes_in_topological_ordering(1)

