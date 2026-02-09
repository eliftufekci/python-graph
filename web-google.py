import matplotlib.pyplot as plt
import random
import networkx as nx
import heapq
import numpy as np
import datetime

class PrefixMap:
    def __init__(self):
        self.map = {}

    def insert(self, path):
        for i in range(len(path.route)):
            prefix = tuple(path.route[:i+1])

            if prefix not in self.map:
                self.map[prefix] = []

            self.map[prefix].append(path)

    def remove(self, path):
        for i in range(len(path.route)):
            prefix = tuple(path.route[:i+1])
            if prefix in self.map:
                try:
                    self.map[prefix].remove(path)
                except ValueError:
                    pass

    def findPathListWithPrefix(self, route):
        return self.map.get(tuple(route), [])

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

def reverse(graph):
    Gr = nx.DiGraph()
    Gr.add_edges_from((v,u,d) for u,v,d in graph.edges(data=True))
    return Gr

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

def ExtendPath(path, graph, graph_state, LQ, global_PQ, covered_vertices, prefix_map):
    tail = path.tail()

    if tail in LQ:
        for p in LQ[tail]:
            if p.cls == path.cls and p.length >= path.length and p.isActive:
                p.isActive = False

    for neighbor in graph[tail]:
        if neighbor not in path.route and neighbor != graph_state.parent.get(tail):
            new_path = path.copy()
            new_path.route.append(neighbor)

            edge_weight = graph[tail][neighbor]['weight']
            new_path.edges[(tail, neighbor)] = edge_weight
            new_path.length += edge_weight
            new_path.lb = new_path.LB1(graph_state)

            class_key = path.cls
            if class_key not in covered_vertices:
                covered_vertices[class_key] = set()

            if neighbor in covered_vertices[class_key]:
                new_path.isActive = False
            else:
                covered_vertices[class_key].add(neighbor)

            if neighbor not in LQ:
                LQ[neighbor] = []
            heapq.heappush(LQ[neighbor], new_path)
            prefix_map.insert(new_path)


    parent = graph_state.parent.get(tail)
    if parent is None:
        return False

    if parent in path.route:
        prefix_map.remove(path)
        return False

    path.route.append(parent)
    edge_weight = graph[tail][parent]['weight']
    path.edges[(tail, parent)] = edge_weight
    path.length += edge_weight

    return True

def AdjustPath(path, global_PQ, LQ, result_set, dest, prefix_map):
    if path.cls is None:
        return

    _, deviation_vertex = path.cls

    updated = False
    for vertex in path.route:
        if vertex in LQ:
            for p in LQ[vertex]:
                if not p.isActive:
                    if p.cls == path.cls:
                        p.isActive = True
                        updated = True

    if path.tail() == dest:
        path_id = len(result_set) + 1

        for i, vertex in enumerate(path.route):
            prefix = path.route[:i+1]

            if len(prefix)>1:
                paths_with_prefix = prefix_map.findPathListWithPrefix(prefix)

                if paths_with_prefix:
                    for p in paths_with_prefix:
                        if len(p.route) > len(prefix):
                            p.cls = (path_id, vertex)

    if updated:
        for i in range(len(global_PQ)):
            temp_queue = global_PQ[i][2]

            if temp_queue:
                first_active = next((p for p in temp_queue if p.isActive), None)
                if first_active:
                    new_key = (not first_active.isActive, first_active.lb)
                    global_PQ[i] = (new_key, global_PQ[i][1], temp_queue)

        heapq.heapify(global_PQ)


def FindNextPath(graph, graph_state, global_PQ, LQ, threshold, result_set, dest, covered_vertices, prefix_map):
    global number_of_paths_explored

    while global_PQ:
        _, _, current_LQ = heapq.heappop(global_PQ)

        if not current_LQ:
            continue

        inactive_paths = [] 
        current_path = None

        while current_LQ:
            candidate = heapq.heappop(current_LQ)

            if candidate.isActive:
                current_path = candidate
                break
            else:
                inactive_paths.append(candidate)

        for p in inactive_paths:
            heapq.heappush(current_LQ, p)

        if current_path is None:
            if current_LQ:
                heapq.heappush(global_PQ, ((not current_LQ[0].isActive, current_LQ[0].lb), id(current_LQ), current_LQ))
            continue

        number_of_paths_explored += 1


        if current_LQ:
            heapq.heappush(global_PQ, ((not current_LQ[0].isActive, current_LQ[0].lb), id(current_LQ), current_LQ))

        while current_path.tail() != dest:
            LB2 = current_path.LB2(threshold=threshold, result_set=result_set)

            if LB2 > current_path.lb:
                current_path.lb = LB2
                AdjustPath(path=current_path, global_PQ=global_PQ, LQ=LQ, result_set=result_set, dest=dest, prefix_map=prefix_map)
                break

            if not ExtendPath(path=current_path, graph=graph, graph_state=graph_state, LQ=LQ, global_PQ=global_PQ, covered_vertices=covered_vertices, prefix_map=prefix_map):
                break

        if current_path.tail() == dest:
            if current_path.cls in covered_vertices:
                covered_vertices[current_path.cls].clear()

            prefix_map.remove(current_path)

            AdjustPath(path=current_path, global_PQ=global_PQ, LQ=LQ, result_set=result_set, dest=dest, prefix_map=prefix_map)

            return current_path

    return None

def average_hop_count(result):
    if not result:
        return 0

    return sum(len(p.route)-1 for p in result) / len(result)

def FindKSPD(graph, graph_reverse, src, dest, k, threshold):
    graph_state = GraphState(graph_reverse=graph_reverse, destination=dest)
    result_set = []
    global_PQ = []
    LQ = {}
    covered_vertices = {}
    prefix_map = PrefixMap()
    global_LQ_ids = set()

    shortest_path = dijkstra(graph=graph, src=src, dest=dest)

    if shortest_path is None:
        print(f"No path exist between {src} and {dest}")
        return []

    result_set.append(shortest_path)
    print("first shortest path found")

    for vertex in shortest_path.route[:-1]:
        for neighbor in graph[vertex]:
            path = Path()
            path.route = shortest_path.route[:shortest_path.route.index(vertex)+1]

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
                for i in range(len(path.route)-1):
                    u, v = path.route[i], path.route[i+1]
                    path.edges[(u,v)] = graph[u][v]['weight']
                    path.length += graph[u][v]['weight']
                path.cls = (1, vertex)
                path.lb = path.LB1(graph_state)

                tail = path.tail()
                if tail not in LQ:
                    LQ[tail] = []
                heapq.heappush(LQ[tail], path)
                prefix_map.insert(path)

                if LQ[tail] and id(LQ[tail]) not in global_LQ_ids:
                    heapq.heappush(global_PQ, ((not LQ[tail][0].isActive, LQ[tail][0].lb), id(LQ[tail]), LQ[tail]))
                    global_LQ_ids.add(id(LQ[tail]))


    while len(result_set) < k and global_PQ:
        new_path = FindNextPath(graph, graph_state, global_PQ, LQ, threshold, result_set, dest, covered_vertices, prefix_map=prefix_map)
        print("New path found")

        if new_path and new_path.Sim(threshold=threshold, result_set=result_set):
            result_set.append(new_path)
            print("Path added to result set")

    return result_set


def visualize_paths_only(G, result_paths, node_names=None):
    # Sadece path'lerdeki node'ları içeren subgraph oluştur
    path_nodes = set()
    for path in result_paths:
        path_nodes.update(path.route)

    subG = G.subgraph(path_nodes).copy()

    plt.figure(figsize=(15, 10))
    pos = nx.spring_layout(subG, k=2, iterations=50)

    # Node'ları çiz
    nx.draw_networkx_nodes(subG, pos, node_size=700, node_color='lightblue')

    # Her path'i farklı renkle çiz
    colors = ['red', 'blue', 'green', 'orange', 'purple']
    for i, path in enumerate(result_paths):
        edges = [(path.route[j], path.route[j+1]) for j in range(len(path.route)-1)]
        nx.draw_networkx_edges(subG, pos, edgelist=edges,
                               edge_color=colors[i % len(colors)],
                               width=3, alpha=0.7, arrows=True,
                               label=f'Path {i+1} (len={path.length})')

    labels = node_names if node_names else {node: str(node) for node in subG.nodes()}
    nx.draw_networkx_labels(subG, pos, labels, font_size=12)

    edge_labels = nx.get_edge_attributes(subG, 'weight')
    nx.draw_networkx_edge_labels(subG, pos, edge_labels, font_size=8)

    plt.legend()
    plt.title("Paths Visualization")
    plt.axis('off')
    plt.tight_layout()
    plt.savefig('paths_only.png', dpi=300, bbox_inches='tight')
    plt.show()


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


    GR = reverse(G)

    print("Finding top-3 shortest paths with diversity...")
    print("Threshold (τ): 0.5")
    print("=" * 60)

    number_of_paths_explored = 0

    start_time = datetime.datetime.now()

    result = FindKSPD(G, GR, src=src, dest=dest, k=10, threshold=0.6)

    end_time = datetime.datetime.now()
    execution_time = end_time-start_time


    for i, path in enumerate(result, 1):
        print(f"\nPath {i}:")
        print(f"  Length: {path.length}")


    print("\n" + "=" * 60)

    print(f"Total paths found: {len(result)}")
    print(f"Number of explored paths: {number_of_paths_explored}")

    average_hop_count = average_hop_count(result)
    print(f"Average Hop Count: {average_hop_count}")

    print(f"Execution time: {execution_time}")

    # visualize_paths_only(G, result)