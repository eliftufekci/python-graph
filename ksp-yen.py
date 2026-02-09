import networkx as nx
import heapq
import random
import datetime

class Path:
    def __init__(self):
        self.route = []
        self.edges = {}
        self.length = 0
        self.lb = 0
        self.cls = None
        self.isActive = True
        self.cached_intersections = {}

    def __str__(self):
        return f"Route: {self.route}, Length: {self.length}, LB: {self.lb}, Class: {self.cls}, isActive: {self.isActive}"

    def __repr__(self):
        return str(self)

    def __lt__(self, other):
        return (not self.isActive, self.lb) < (not other.isActive, other.lb)

    def tail(self):
        return self.route[-1] if self.route else None

    def head(self):
        return self.route[0] if self.route else None

    def copy(self):
        new_path = Path()
        new_path.route = self.route.copy()
        new_path.edges = self.edges.copy()
        new_path.length = self.length
        new_path.lb = self.lb
        new_path.cls = self.cls
        new_path.isActive = self.isActive
        new_path.cached_intersections = self.cached_intersections.copy()
        return new_path

    def LB1(self, graph_state):
        tail = self.tail()
        if tail is None:
            return 0

        if not graph_state.isSettled[tail]:
            ConstructPartialSPT(graph_state=graph_state, v=tail)

        return self.length + graph_state.distances[tail]

    def LB2(self, threshold, result_set):
        if not result_set:
            return 0

        lb2 = 0
        for old_path in result_set:
            if id(old_path) in self.cached_intersections:
                intersection_length = self.cached_intersections[id(old_path)] 
            
            else:
                common_edges = set(old_path.edges.keys()).intersection(set(self.edges.keys()))
                intersection_length = sum(old_path.edges[e] for e in common_edges)
                self.cached_intersections[id(old_path)] = intersection_length

            current_lb2 = intersection_length * (1 + 1/threshold) - old_path.length
            lb2 = max(lb2, current_lb2)

        return lb2


    def Sim(self, threshold, result_set):
        for old_path in result_set:
            common_edges = set(old_path.edges.keys()).intersection(set(self.edges.keys()))
            intersection_length = sum(old_path.edges[e] for e in common_edges)
            union_length = self.length + old_path.length - intersection_length

            if union_length > 0:
                similarity = intersection_length / union_length
                if similarity > threshold:
                    return False
        return True

def dijkstra(graph, src, dest):
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

def build_path_from_route(graph, route):
    p = Path()
    p.route = route.copy()

    for i in range(len(route)-1):
        u, v = route[i], route[i+1]
        w = graph[u][v]['weight']
        p.edges[(u, v)] = w
        p.length += w

    p.lb = p.length
    return p


def FindKSP(graph, src, dest, k):

    A = []  # shortest paths found
    B = []  # candidate paths (min-heap)

    first_path = dijkstra(graph, src, dest)
    if first_path is None:
        return []

    A.append(first_path)

    for i in range(1, k):
        prev_path = A[i - 1]

        for spur_index in range(len(prev_path.route) - 1):
            spur_node = prev_path.route[spur_index]
            root_route = prev_path.route[:spur_index + 1]

            # Copy graph
            G_copy = graph.copy()

            # Remove edges that recreate previous paths
            for p in A:
                if len(p.route) > spur_index and p.route[:spur_index + 1] == root_route:
                    u = p.route[spur_index]
                    v = p.route[spur_index + 1]
                    if G_copy.has_edge(u, v):
                        G_copy.remove_edge(u, v)

            # Remove nodes in root path except spur node
            for node in root_route[:-1]:
                if node in G_copy:
                    G_copy.remove_node(node)

            spur_path = dijkstra(G_copy, spur_node, dest)
            if spur_path is None:
                continue

            # Combine root + spur
            total_route = root_route[:-1] + spur_path.route
            total_path = build_path_from_route(graph, total_route)

            heapq.heappush(B, (total_path.length, total_path))

        if not B:
            break

        _, next_path = heapq.heappop(B)
        A.append(next_path)

    return A


if __name__ == "__main__":
    G = nx.DiGraph()

    with open("/content/sample_data/web-Google.txt") as f:
        for line in f:
            u,v = map(int, line.split())
            G.add_edge(u,v,weight=1)

    src = random.choice(list(G.nodes()))
    reachable = nx.descendants(G, src)
    
    while not reachable:
        src = random.choice(list(G.nodes()))
        reachable = nx.descendants(G, src)

    dest = random.choice(list(reachable))
    print(src,dest)


    print("Finding top-3 shortest paths with diversity...")
    print("=" * 60)

    number_of_paths_explored = 0

    start_time = datetime.datetime.now()

    result = FindKSP(G, src=src, dest=dest, k=10)

    end_time = datetime.datetime.now()
    execution_time = end_time-start_time


    for i, path in enumerate(result, 1):
        print(f"\nPath {i}:")
        print(f"  Length: {path.length}")


    print(f"Execution time: {execution_time}")