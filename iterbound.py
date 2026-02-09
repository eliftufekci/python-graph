import networkx as nx
import heapq
import random
import datetime

class GraphState:
    def __init__(self, graph_reverse, destination):
        self.graph_reverse = graph_reverse
        self.destination = destination
        self.distances = {}
        self.isSettled = {}
        self.parent = {}
        self.PQ = []

        for node in graph_reverse.nodes():
            self.distances[node] = float('inf')
            self.isSettled[node] = False
            self.parent[node] = None

        heapq.heappush(self.PQ, (0, destination))
        self.distances[destination] = 0

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

def reverse(graph):
    Gr = nx.DiGraph()
    Gr.add_edges_from((v,u,d) for u,v,d in graph.edges(data=True))
    return Gr

def ConstructPartialSPT(graph_state, v):
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


def FindIterBound(G, GR, src, dest, k):

    graph_state = GraphState(graph_reverse=GR, destination=dest)

    results = []
    explored = 0

    first_path = dijkstra(G, src, dest)
    if first_path is None:
        return []

    results.append(first_path)

    # Upper bound = length of k-th path (unknown yet)
    upper_bound = float('inf')

    PQ = []
    heapq.heappush(PQ, (first_path.lb, first_path))

    while PQ and len(results) < k:
        _, path = heapq.heappop(PQ)
        explored += 1

        # Bounding
        if path.lb >= upper_bound:
            continue

        # Expand path
        last = path.route[-1]

        for nbr in G.neighbors(last):
            if nbr in path.route:
                continue  # loop avoidance

            new_path = path.clone()
            w = G[last][nbr]['weight']

            new_path.route.append(nbr)
            new_path.edges[(last, nbr)] = w
            new_path.length += w

            # LB1: optimistic bound
            if nbr not in graph_state.distances:
                continue

            ConstructPartialSPT(graph_state, nbr)
            new_path.lb = new_path.length + graph_state.distances[nbr]

            # Bound check
            if new_path.lb >= UB:
                continue

            if nbr == dest:
                results.append(new_path)
                results.sort(key=lambda p: p.length)

                if len(results) == k:
                    UB = results[-1].length
            else:
                heapq.heappush(PQ, (new_path.lb, new_path))

    print(f"IterBound explored paths: {explored}")
    return results

import networkx as nx
import heapq

def reverse(graph):
    Gr = nx.DiGraph()
    Gr.add_edges_from((v,u,d) for u,v,d in graph.edges(data=True))
    return Gr

def dijkstra_all(G, src):

    dist = {src: 0}
    pq = [(0, src)]

    while pq:
        d, u = heapq.heappop(pq)
        if d > dist[u]:
            continue


        for v in G.neighbors(u):
            w = G[u][v]['weight']
            nd = d + w

            if v not in dist or nd < dist[v]:
                dist[v] = nd
                heapq.heappush(pq, (nd, v))

    return dist

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
GR = reverse(G)
dijkstra(G, 1, 4)

FindIterBound()


#-----------------------------------------