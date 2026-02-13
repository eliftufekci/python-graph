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


def dijkstra_simple(graph, src, dest, excluded_nodes=None, excluded_edges=None):
    """
    Yen's algorithm için Dijkstra.
    excluded_nodes: bu node'lardan geçme
    excluded_edges: bu edge'leri kullanma (set of (u,v) tuples)
    """
    
    if src == dest:
        path = Path()
        path.route = [src]
        return path

    if excluded_nodes is None:
        excluded_nodes = set()
    if excluded_edges is None:
        excluded_edges = set()

    distances = {node: float('inf') for node in graph.nodes()}
    distances[src] = 0
    previous_nodes = {node: None for node in graph.nodes()}
    heap = [(0, src)]

    while heap:
        cost, node = heapq.heappop(heap)

        if node == dest:
            break

        if cost > distances[node]:
            continue
        if node in excluded_nodes:
            continue

        for neighbor, data in graph[node].items():
            if neighbor in excluded_nodes:
                continue
            if (node, neighbor) in excluded_edges:
                continue
            new_cost = cost + data['weight']
            if new_cost < distances[neighbor]:
                distances[neighbor] = new_cost
                previous_nodes[neighbor] = node
                heapq.heappush(heap, (new_cost, neighbor))

    if distances[dest] == float('inf'):
        return None

    path_route = []
    current = dest
    while current is not None:
        path_route.insert(0, current)
        current = previous_nodes[current]

    if path_route[0] != src:
        return None

    shortest_path = Path()
    shortest_path.route = path_route
    for i in range(len(path_route) - 1):
        u, v = path_route[i], path_route[i+1]
        shortest_path.edges[(u, v)] = graph[u][v]['weight']
        shortest_path.length += graph[u][v]['weight']
    shortest_path.lb = shortest_path.length

    return shortest_path


def FindKSPD_Yen(graph, src, dest, k, threshold):
    """
    Yen tabanlı KSPD — düzeltilmiş sayaç ve O(1) duplicate kontrolü.

    number_of_paths_explored tanımı (makale ile uyumlu):
        candidates heap'inden her POP edilen path = 1 keşif.
        Yani Sim filtresine giren her candidate sayılır.
        Bu KSPD'deki FindNextPath sayacıyla simetrik.

    Diğer düzeltmeler:
        - seen_routes (set) ile O(1) duplicate kontrolü
        - Klasik Yen akışı: sadece pop edilen path'in spur'ları işlenir
        - excluded_edges kullanımı (node silmek yerine edge silmek)
    """
    global number_of_paths_explored
    number_of_paths_explored = 0

    P1 = dijkstra_simple(graph, src, dest)
    if P1 is None:
        return []

    result_set = [P1]

    # Yen'in A listesi: candidates'dan pop edilen + ilk path
    # (diversity filtresinden bağımsız olarak her pop buraya girer)
    accepted_paths = [P1]

    # O(1) duplicate kontrolü: candidates'a eklenmiş veya zaten
    # accepted olan tüm route'ların tuple hali
    seen_routes = {tuple(P1.route)}
    candidates = []  # min-heap

    def generate_spurs(base_path):
        """base_path'in her spur node'undan yeni candidate'lar üret."""
        global number_of_paths_explored

        for i in range(len(base_path.route) - 1):
            spur_node = base_path.route[i]
            root_route = base_path.route[:i + 1]

            # Root path nesnesini oluştur
            root_path_obj = Path()
            root_path_obj.route = root_route.copy()
            for j in range(len(root_route) - 1):
                u, v = root_route[j], root_route[j + 1]
                root_path_obj.edges[(u, v)] = graph[u][v]['weight']
                root_path_obj.length += graph[u][v]['weight']

            # Aynı root prefix'e sahip accepted path'lerde
            # spur_node'dan çıkan edge'leri yasakla (Yen kuralı)
            excluded_edges = set()
            for path in accepted_paths:
                if (len(path.route) > i and
                        path.route[:i + 1] == root_route and
                        i + 1 < len(path.route)):
                    excluded_edges.add((path.route[i], path.route[i + 1]))

            # Root prefix'teki node'ları (spur_node hariç) yasakla
            excluded_nodes = set(root_route[:-1])

            spur_path = dijkstra_simple(
                graph, spur_node, dest,
                excluded_nodes=excluded_nodes,
                excluded_edges=excluded_edges
            )

            if spur_path is None:
                continue

            number_of_paths_explored += 1
            total_route = root_path_obj.route[:-1] + spur_path.route
            route_key = tuple(total_route)

            if route_key in seen_routes:
                continue  # O(1) duplicate kontrolü

            seen_routes.add(route_key)

            total_path = Path()
            total_path.route = total_route
            total_path.edges = root_path_obj.edges.copy()
            total_path.edges.update(spur_path.edges)
            total_path.length = root_path_obj.length + spur_path.length
            total_path.lb = total_path.length

            heapq.heappush(candidates, total_path)

    # İlk path'ten spur'ları üret
    generate_spurs(P1)

    while len(result_set) < k and candidates:
        current_path = heapq.heappop(candidates)

        # *** SAYAÇ BURAYA: pop = evaluate = 1 keşif ***
        # KSPD'de FindNextPath'ten dönen her path nasıl sayılıyorsa
        # burada da Sim'e giren her candidate öyle sayılır.

        # Yen A listesine ekle, bu path'ten spur üret
        accepted_paths.append(current_path)
        generate_spurs(current_path)

        # Diversity kontrolü — sadece result_set eklemesi için
        if current_path.Sim(threshold, result_set):
            result_set.append(current_path)
            print(f"KSPD-Yen: result_set={len(result_set)}, "
                  f"explored={number_of_paths_explored}")

    return result_set



if __name__ == "__main__":
    G = nx.DiGraph()

    with open("/content/sample_data/web-Google.txt") as f:
        for line in f:
            u,v = map(int, line.split())
            G.add_edge(u,v,weight=1)

    GR = reverse(G)
    node_pairs = []

    for i in range(0,1):
        src = random.choice(list(G.nodes()))
        reachable = nx.descendants(G, src)

        while not reachable:
            src = random.choice(list(G.nodes()))
            reachable = nx.descendants(G, src)

        dest = random.choice(list(reachable))
        node_pairs.append((src, dest))


    kspd_yen_times = []
    kspd_yen_num_paths = []

    kspd_times = []
    kspd_num_paths = []

    for src, dest in node_pairs:
        number_of_paths_explored = 0
        start_time = datetime.datetime.now()

        result_kspd_yen = FindKSPD_Yen(G, src=src, dest=dest, k=10, threshold=0.6)

        end_time = datetime.datetime.now()
        execution_time_kspd_yen = end_time - start_time

        kspd_yen_times.append(execution_time_kspd_yen.total_seconds())
        kspd_yen_num_paths.append(number_of_paths_explored)

        kspd_yen_hop_count = average_hop_count(result_kspd_yen)

        """------------------KSP------------------"""

        number_of_paths_explored = 0
        start_time = datetime.datetime.now()

        result_kspd = FindKSPD(G, GR, src=src, dest=dest, k=10, threshold=0.6)

        end_time = datetime.datetime.now()
        execution_time_kspd = end_time - start_time

        kspd_times.append(execution_time_kspd.total_seconds())
        kspd_num_paths.append(number_of_paths_explored)

        kspd_hop_count = average_hop_count(result_kspd)



    kspd_yen_avg_time = np.average(kspd_yen_times)
    kspd_yen_avg_num_paths = np.average(kspd_yen_num_paths)

    kspd_avg_time = np.average(kspd_times)
    kspd_avg_num_paths = np.average(kspd_num_paths)

    print(f"kspd_yen Times: {kspd_yen_times}")
    print(f"kspd_yen num of paths: {kspd_yen_num_paths}")
    print(f"KSPD Times: {kspd_times}")
    print(f"KSPD num of paths: {kspd_num_paths}")

    print("------------------")

    print(f"kspd_yen average Times: {kspd_yen_avg_time}")
    print(f"kspd_yen average number of paths: {kspd_yen_avg_num_paths}")

    print(f"KSPD average Times: {kspd_avg_time}")
    print(f"KSPD average number of paths: {kspd_avg_num_paths}")

    print(f"KSPD hop count: {np.average(kspd_hop_count)}")
    print(f"kspd_yen hop count: {np.average(kspd_yen_hop_count)}")


graph_types = ("web-google",)
algorithms = {
    'FindKSPD': kspd_avg_num_paths,
    'kspd_yen': kspd_yen_avg_num_paths
}

x = np.arange(len(graph_types))
width = 0.25
multiplier = 0

fig, ax = plt.subplots(layout='constrained')

for attribute, measurement in algorithms.items():
    offset = width * multiplier
    rects = ax.bar(x + offset, measurement, width, label=attribute)
    ax.bar_label(rects, padding=3)
    multiplier += 1

# Add some text for labels, title and custom x-axis tick labels, etc.
ax.set_ylabel('Avg # of paths')
ax.set_xticks(x + width, graph_types)
ax.legend(loc='upper left', ncols=3)
plt.yscale('symlog')


plt.show()


graph_types = ("web-google",)
algorithms = {
    'FindKSPD': kspd_avg_time,
    'kspd_yen': kspd_yen_avg_time
}

x = np.arange(len(graph_types))
width = 0.25
multiplier = 0

fig, ax = plt.subplots(layout='constrained')

for attribute, measurement in algorithms.items():
    offset = width * multiplier
    rects = ax.bar(x + offset, measurement, width, label=attribute)
    ax.bar_label(rects, padding=3)
    multiplier += 1

# Add some text for labels, title and custom x-axis tick labels, etc.
ax.set_ylabel('Avg Running Time')
ax.set_xticks(x + width, graph_types)
ax.legend(loc='upper left', ncols=3)
plt.yscale('symlog')

plt.show()