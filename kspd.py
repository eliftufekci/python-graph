import networkx as nx
import heapq

class GraphState:
    """Manages reverse shortest path tree state"""
    def __init__(self, graph_reverse, destination):
        self.graph_reverse = graph_reverse
        self.destination = destination
        self.distances = {}
        self.isSettled = {}
        self.parent = {}
        self.PQ = []
        
        # Initialize
        for node in graph_reverse.nodes():
            self.distances[node] = float('inf')
            self.isSettled[node] = False
            self.parent[node] = None
        
        heapq.heappush(self.PQ, (0, destination))
        self.distances[destination] = 0


class Path:
    """Represents a path in the graph"""
    def __init__(self):
        self.route = []
        self.edges = {}
        self.length = 0
        self.lb = 0
        self.cls = None
        self.isActive = True
    
    def __str__(self):
        return f"Route: {self.route}, Length: {self.length}, LB: {self.lb}, Class: {self.cls}"
    
    def __lt__(self, other):
        return self.lb < other.lb
    
    def tail(self):
        """Get the last vertex in the path"""
        return self.route[-1] if self.route else None
    
    def head(self):
        """Get the first vertex in the path"""
        return self.route[0] if self.route else None
    
    def contains(self, vertex):
        """Check if vertex is in the path"""
        return vertex in self.route
    
    def copy(self):
        """Create a deep copy of the path"""
        new_path = Path()
        new_path.route = self.route.copy()
        new_path.edges = self.edges.copy()
        new_path.length = self.length
        new_path.lb = self.lb
        new_path.cls = self.cls
        new_path.isActive = self.isActive
        return new_path
    
    def compute_LB1(self, graph_state):
        """Compute shortest path lower bound (LB1)"""
        tail = self.tail()
        if tail is None:
            return 0
        
        # Lazily construct partial SPT if needed
        if not graph_state.isSettled[tail]:
            ConstructPartialSPT(graph_state, tail)
        
        return self.length + graph_state.distances[tail]
    
    def compute_LB2(self, threshold, result_set):
        """Compute diversified path lower bound (LB2)"""
        if not result_set:
            return 0
        
        lb2 = 0
        for old_path in result_set:
            common_edges = set(old_path.edges.keys()).intersection(set(self.edges.keys()))
            intersection_length = sum(old_path.edges[e] for e in common_edges)
            
            current_lb2 = intersection_length * (1 + 1/threshold) - old_path.length
            lb2 = max(lb2, current_lb2)
        
        return lb2
    
    def check_similarity(self, threshold, result_set):
        """Check if path is dissimilar enough from all paths in result set"""
        for old_path in result_set:
            common_edges = set(old_path.edges.keys()).intersection(set(self.edges.keys()))
            intersection_length = sum(old_path.edges[e] for e in common_edges)
            union_length = self.length + old_path.length - intersection_length
            
            if union_length > 0:
                similarity = intersection_length / union_length
                if similarity > threshold:
                    return False
        return True


def ConstructPartialSPT(graph_state, v):
    """Incrementally construct reverse shortest path tree until vertex v is settled"""
    if graph_state.isSettled[v]:
        return graph_state.distances[v]
    
    while graph_state.PQ:
        cost, node = heapq.heappop(graph_state.PQ)
        
        if cost > graph_state.distances[node]:
            continue
        
        if not graph_state.isSettled[node]:
            graph_state.isSettled[node] = True
            
            for neighbor, data in graph_state.graph_reverse[node].items():
                if not graph_state.isSettled[neighbor]:
                    new_cost = cost + data['weight']
                    
                    if new_cost < graph_state.distances[neighbor]:
                        graph_state.distances[neighbor] = new_cost
                        graph_state.parent[neighbor] = node
                        heapq.heappush(graph_state.PQ, (new_cost, neighbor))
            
            if node == v:
                return graph_state.distances[v]
    
    return float('inf')


def reverse_graph(graph):
    """Create reverse graph"""
    Gr = nx.DiGraph()
    Gr.add_edges_from((v, u, d) for u, v, d in graph.edges(data=True))
    return Gr


def dijkstra(graph, src, dest):
    """Standard Dijkstra to find shortest path"""
    if src == dest:
        path = Path()
        path.route = [src]
        return path
    
    heap = [(0, src, [])]
    visited = set()
    
    while heap:
        cost, node, path_list = heapq.heappop(heap)
        
        if node in visited:
            continue
        visited.add(node)
        
        if node == dest:
            shortest_path = Path()
            
            for i in range(len(path_list)):
                if i < len(path_list) - 1:
                    u, v = path_list[i], path_list[i+1]
                    shortest_path.edges[(u, v)] = graph[u][v]['weight']
                    shortest_path.length += graph[u][v]['weight']
                    shortest_path.route.append(u)
                else:
                    u, v = path_list[i], dest
                    shortest_path.edges[(u, v)] = graph[u][v]['weight']
                    shortest_path.length += graph[u][v]['weight']
                    shortest_path.route.append(u)
                    shortest_path.route.append(v)
            
            shortest_path.lb = shortest_path.length
            return shortest_path
        
        for neighbor, data in graph[node].items():
            if neighbor not in visited:
                new_cost = cost + data['weight']
                heapq.heappush(heap, (new_cost, neighbor, path_list + [node]))
    
    return None


def ExtendPath(path, graph, graph_state, LQ, global_PQ, covered_vertices):
    """Extend path at its tail vertex (Algorithm 5)"""
    tail = path.tail()
    
    # Mark dominated paths as inactive (lines 2-3)
    if tail in LQ:
        for p in LQ[tail]:
            if p.cls == path.cls and p.length >= path.length and p.isActive:
                p.isActive = False
    
    # Try extending via neighbors (lines 4-10)
    for neighbor in graph[tail]:
        # Check if neighbor creates a valid extension
        if neighbor not in path.route and neighbor != graph_state.parent.get(tail):
            new_path = path.copy()
            new_path.route.append(neighbor)
            
            edge_weight = graph[tail][neighbor]['weight']
            new_path.edges[(tail, neighbor)] = edge_weight
            new_path.length += edge_weight
            new_path.lb = new_path.compute_LB1(graph_state)
            
            # Check if vertex already covered by this class (lines 6-9)
            class_key = path.cls
            if class_key not in covered_vertices:
                covered_vertices[class_key] = set()
            
            if neighbor in covered_vertices[class_key]:
                new_path.isActive = False
            else:
                covered_vertices[class_key].add(neighbor)
            
            # Add to local queue (lines 9-10)
            if neighbor not in LQ:
                LQ[neighbor] = []
            heapq.heappush(LQ[neighbor], new_path)
            
            if LQ[neighbor]:
                heapq.heappush(global_PQ, (LQ[neighbor][0].lb, id(LQ[neighbor]), LQ[neighbor]))
    
    # Try following parent in reverse SPT (lines 11-16)
    parent = graph_state.parent.get(tail)
    if parent is None:
        return False
    
    if parent in path.route:
        return False
    
    # Extend along shortest path tree
    path.route.append(parent)
    edge_weight = graph[tail][parent]['weight']
    path.edges[(tail, parent)] = edge_weight
    path.length += edge_weight
    
    return True


def AdjustPath(path, LQ, result_set, dest):
    """Activate dominated paths and update classifications (Algorithm 6)"""
    if path.cls is None:
        return
    
    _, deviation_vertex = path.cls
    
    # Activate dominated paths (line 3)
    for vertex in path.route:
        if vertex in LQ:
            for p in LQ[vertex]:
                if not p.isActive:
                    # Check if this path was dominated by current path
                    if p.cls == path.cls and p.length >= path.length:
                        p.isActive = True
    
    # Update classifications if path reached destination (line 3 continued)
    if path.tail() == dest:
        path_id = len(result_set) + 1
        
        for vertex in path.route:
            if vertex in LQ:
                for p in LQ[vertex]:
                    # Check if path has the prefix
                    if len(p.route) > 0:
                        # Find if vertex is in p's route
                        try:
                            vertex_idx = path.route.index(vertex)
                            if len(p.route) >= vertex_idx + 1:
                                if p.route[:vertex_idx + 1] == path.route[:vertex_idx + 1]:
                                    p.cls = (path_id, vertex)
                        except ValueError:
                            continue


def FindNextPath(graph, graph_state, global_PQ, LQ, threshold, result_set, dest, covered_vertices):
    """Find next promising path (Algorithm 4)"""
    while global_PQ:
        # Extract path with minimum lower bound (line 2)
        _, _, current_LQ = heapq.heappop(global_PQ)
        
        if not current_LQ:
            continue
        
        current_path = heapq.heappop(current_LQ)
        
        # Re-add local queue if not empty (line 3)
        if current_LQ:
            heapq.heappush(global_PQ, (current_LQ[0].lb, id(current_LQ), current_LQ))
        
        # Extend path until destination or infeasible (line 4)
        while current_path.tail() != dest:
            # Compute LB2 (line 5)
            lb2 = current_path.compute_LB2(threshold, result_set)
            
            # Check if we need to postpone this path (lines 6-11)
            if lb2 > current_path.lb:
                current_path.lb = lb2
                AdjustPath(current_path, LQ, result_set, dest)
                
                heapq.heappush(current_LQ, current_path)
                if current_LQ:
                    heapq.heappush(global_PQ, (current_LQ[0].lb, id(current_LQ), current_LQ))
                break
            
            # Try to extend path (line 12)
            if not ExtendPath(current_path, graph, graph_state, LQ, global_PQ, covered_vertices):
                break
        
        # Check if we reached destination (lines 14-16)
        if current_path.tail() == dest:
            AdjustPath(current_path, LQ, result_set, dest)
            return current_path
    
    return None


def FindKSPD(graph, graph_reverse, src, dest, k, threshold):
    """Main algorithm to find k shortest paths with diversity (Algorithm 3)"""
    # Initialize graph state (line 1)
    graph_state = GraphState(graph_reverse, dest)
    
    # Result set
    result_set = []
    
    # Find first shortest path (lines 2-3)
    shortest_path = dijkstra(graph, src, dest)
    if shortest_path is None:
        return result_set
    
    result_set.append(shortest_path)
    
    # Initialize queues (line 1)
    global_PQ = []  # Priority queue of local queues
    LQ = {}  # Local queues for each vertex
    covered_vertices = {}  # Track which vertices are covered by which class
    
    # Generate initial deviation paths (line 4)
    for vertex in shortest_path.route[:-1]:
        for neighbor in graph[vertex]:
            # Create deviation path
            path = Path()
            path.route = shortest_path.route[:shortest_path.route.index(vertex) + 1]
            
            # Check if this creates a valid new path
            should_add = False
            
            if neighbor not in path.route:
                if neighbor in shortest_path.route:
                    next_idx = shortest_path.route.index(vertex) + 1
                    if next_idx < len(shortest_path.route) and shortest_path.route[next_idx] != neighbor:
                        path.route.append(neighbor)
                        should_add = True
                else:
                    path.route.append(neighbor)
                    should_add = True
            
            if should_add:
                # Compute path properties
                for j in range(len(path.route) - 1):
                    u, v = path.route[j], path.route[j + 1]
                    path.edges[(u, v)] = graph[u][v]['weight']
                    path.length += graph[u][v]['weight']
                
                path.cls = (1, vertex)  # (path_id, deviation_vertex)
                path.lb = path.compute_LB1(graph_state)
                
                # Add to local queue
                tail = path.tail()
                if tail not in LQ:
                    LQ[tail] = []
                heapq.heappush(LQ[tail], path)
                
                # Add local queue to global queue
                if LQ[tail]:
                    heapq.heappush(global_PQ, (LQ[tail][0].lb, id(LQ[tail]), LQ[tail]))
    
    # Main loop - find remaining k-1 paths (lines 5-8)
    while len(result_set) < k and global_PQ:
        # Find next path (line 6)
        new_path = FindNextPath(graph, graph_state, global_PQ, LQ, threshold, result_set, dest, covered_vertices)
        
        # Check if path is feasible (line 7-8)
        if new_path and new_path.check_similarity(threshold, result_set):
            result_set.append(new_path)
    
    return result_set


# ============================================================================
# EXAMPLE USAGE
# ============================================================================

if __name__ == "__main__":
    # Create graph from Figure 1 in the paper
    G = nx.DiGraph()
    edges = [
        (1, 2, 10),   # A -> B
        (2, 3, 1),    # B -> C
        (3, 4, 10),   # C -> D
        (1, 8, 20),   # A -> I
        (8, 2, 1),    # I -> B
        (2, 6, 1),    # B -> F
        (6, 7, 1),    # F -> H
        (7, 8, 1),    # H -> I
        (2, 7, 3),    # B -> H
        (7, 5, 15),   # H -> E
        (3, 5, 18),   # C -> E
        (5, 4, 1)     # E -> D
    ]
    
    # Node mapping: A=1, B=2, C=3, D=4, E=5, F=6, H=7, I=8
    G.add_weighted_edges_from(edges)
    
    # Create reverse graph
    GR = reverse_graph(G)
    
    # Run algorithm
    print("Finding top-3 shortest paths with diversity...")
    print("Source: A (1), Destination: D (4)")
    print("Threshold (τ): 0.5")
    print("=" * 60)
    
    result = FindKSPD(G, GR, src=1, dest=4, k=3, threshold=0.5)
    
    # Print results
    node_names = {1: 'A', 2: 'B', 3: 'C', 4: 'D', 5: 'E', 6: 'F', 7: 'H', 8: 'I'}
    
    for i, path in enumerate(result, 1):
        route_str = ' -> '.join(node_names[v] for v in path.route)
        print(f"\nPath {i}:")
        print(f"  Route: {route_str}")
        print(f"  Length: {path.length}")
        
        # Calculate similarity with other paths
        if i > 1:
            for j in range(i - 1):
                old_path = result[j]
                common = set(old_path.edges.keys()).intersection(set(path.edges.keys()))
                inter_len = sum(old_path.edges[e] for e in common)
                union_len = path.length + old_path.length - inter_len
                sim = inter_len / union_len if union_len > 0 else 0
                print(f"  Similarity with Path {j+1}: {sim:.2f}")
    
    print("\n" + "=" * 60)
    print(f"Total paths found: {len(result)}")
