import networkx as nx
import heapq
import pandas as pd

def print_global(pq):
    c = pq.copy()
    while c:
        print(heapq.heappop(c))

def print_local(pq):
    c = pq.copy()
    while c:
        print(heapq.heappop(c))

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
            common_edges = set(old_path.edges.keys()).intersection(set(self.edges.keys()))
            intersection_length = sum(old_path.edges[e] for e in common_edges)

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

def ExtendPath(path, graph, graph_state, LQ, global_PQ, covered_vertices):
    print("Extend pathe geldim")
    print(f"işlediğim path {path}")
    tail = path.tail()

    if tail in LQ:
        for p in LQ[tail]:
            if p.cls == path.cls and p.length >= path.length and p.isActive:
                print(f"p pathi: {p} ile extend edeceğim path {path} aynı sınıftalar ve extend edilen daha kısa olduğu için p yi inactive yapıyoruz")
                p.isActive = False

    for neighbor in graph[tail]:
        if neighbor not in path.route and neighbor != graph_state.parent.get(tail):
            print("eğer neighbor rotada yoksa ve benim parentim değil o yüzden uzatıyorum")
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
                print("eğer neighbor'un class keyi covered vertices içinde o yüzden yol inactive olmalı")
                new_path.isActive = False
            else:
                covered_vertices[class_key].add(neighbor)

            if neighbor not in LQ:
                LQ[neighbor] = []
            heapq.heappush(LQ[neighbor], new_path)

            print("LQ ya newpathi push ettim")
            for x in LQ.values():
                print_local(x)

            print("global_PQ nun durumu")
            print_global(global_PQ)

            # # burada LQ da yaptığım değişiklik zaten global olanda görünüyor, tekrar pushlamam aynı şeyi 2 defa pushlamam anlamına geliyor
            # if LQ[neighbor]:
            #     heapq.heappush(global_PQ, ((not LQ[neighbor][0].isActive, LQ[neighbor][0].lb), id(LQ[neighbor]), LQ[neighbor]))
            #     print("global_PQ nun push sonrası durumu")
            #     print_global(global_PQ)

    parent = graph_state.parent.get(tail)
    if parent is None:
        return False

    if parent in path.route:
        print("Cycle var, uzatamadık")
        return False

    path.route.append(parent)
    edge_weight = graph[tail][parent]['weight']
    path.edges[(tail, parent)] = edge_weight
    path.length += edge_weight
    print(f"genişletmemi yaptım path şu hale geldi: {path}")

    return True

def AdjustPath(path, LQ, result_set, dest):
    print("Adjust path'e geldim")

    print("LQ'nun durumu")
    for x in LQ.keys():
        print_local(LQ[x])


    if path.cls is None:
        return

    _, deviation_vertex = path.cls

    for vertex in path.route:
        if vertex in LQ:
            for p in LQ[vertex]:
                if not p.isActive:
                    print(f"{p} pathi domine edilmiş, onu activate yapıyoruz")
                    # Check if this path was dominated by current path
                    if p.cls == path.cls:
                        p.isActive = True

    if path.tail() == dest:
        print(f"{path}, destinationa ulaşmış, prefix güncellemesi yapıcaz")
        path_id = len(result_set) + 1

        for lq in LQ.values():
            for p in lq:
                for vertex in path.route:
                    try:
                        path_index = path.route.index(vertex)
                        p_index = p.route.index(vertex)
                        if p.route[:p_index + 1] == path.route[:path_index + 1] and len(p.route[:p_index + 1])>1:
                            print(f"p pathi: {p}, bulunan yeni path: {path} ile aynı prefixe sahip")
                            print(f"{p.route[:p_index + 1]} ile {path.route[:path_index + 1]} aynı yani")
                            print(f"class'ı {(path_id, vertex)} olarak değiştiriyoruz")
                            p.cls = (path_id, vertex)
                    except ValueError:
                        continue


def FindNextPath(graph, graph_state, global_PQ, LQ, threshold, result_set, dest, covered_vertices):
    global number_of_paths_explored

    print("Find next pathe geldim")
    print("Find Next path başında global_PQ nun durumu")
    print_global(global_PQ)

    print("LQ nun durumu: ")
    for x in LQ.keys():
        print_local(LQ[x])


    while global_PQ:
        _, _, current_LQ = heapq.heappop(global_PQ)

        print("Sırayla her iterasyonda current lq içindeki pathler")
        print_local(current_LQ)
        
        if not current_LQ:
            continue
        
        current_path = current_LQ[0]            
        number_of_paths_explored += 1

        print(f"Find Next Path içinde o an işlenen path: {current_path}")

        if not current_path.isActive:
            print("path inactive olduğu için geçiyoruz")
            print( "local queue hali:")
            print_local(current_LQ)
            heapq.heappush(global_PQ, ((not current_LQ[0].isActive, current_LQ[0].lb), id(current_LQ), current_LQ))
            print("global queue hali:")
            print_global(global_PQ)
            continue
        
        current_path = heapq.heappop(current_LQ)          

        if current_LQ:
            heapq.heappush(global_PQ, ((not current_LQ[0].isActive, current_LQ[0].lb), id(current_LQ), current_LQ))
            print("o an işlenen path çıkarıldıktan sonra local queue'da hala eleman varsa local queue'nun sonr halini tekrar global_PQ ya push ediyoruz: ")
            print_global(global_PQ)
            
        
        while current_path.tail() != dest:
            print("tail dest e ulaşana kadar while içindeyiz")

            LB2 = current_path.LB2(threshold=threshold, result_set=result_set)
            
            if LB2 > current_path.lb:
                current_path.lb = LB2
                print("lb2 değeri büyük çıktı adjust path yapıcaz")
                AdjustPath(path=current_path, LQ=LQ, result_set=result_set, dest=dest)
                print(f"adjust path yaptıktan sonra elimizdeki current path {current_path}, bunu current_LQ ya push etmemiz lazım")
                heapq.heappush(current_LQ, current_path)
                if current_LQ:
                    heapq.heappush(global_PQ, ((not current_LQ[0].isActive, current_LQ[0].lb), id(current_LQ), current_LQ))
                    print_global(global_PQ)
                break

            if not ExtendPath(path=current_path, graph=graph, graph_state=graph_state, LQ=LQ, global_PQ=global_PQ, covered_vertices=covered_vertices):
                break

        if current_path.tail() == dest:
            print(f"current path: {current_path}, destinationa ulaştı, adjust path yapıcaz")
            AdjustPath(path=current_path, LQ=LQ, result_set=result_set, dest=dest)
            return current_path
    
    return None

def FindKSPD(graph, graph_reverse, src, dest, k, threshold):
    graph_state = GraphState(graph_reverse=graph_reverse, destination=dest)
    result_set = []
    global_PQ = []
    LQ = {}
    covered_vertices = {}  # Track which vertices are covered by which class

    shortest_path = dijkstra(graph=graph, src=src, dest=dest)
    result_set.append(shortest_path)
    print(f"fisrt shortest path: {shortest_path.route}")

    for vertex in shortest_path.route[:-1]:
        # print("----")
        # print(f"vertex: {vertex}")

        for neighbor in graph[vertex]:
            path = Path()
            path.route = shortest_path.route[:shortest_path.route.index(vertex)+1]
            # print(f"path route: {path.route}")
            # print(f"path length: {path.length}")

            should_add = False
            # print(f"neighbor: {neighbor}")

            if neighbor not in path.route:
                if neighbor in shortest_path.route:
                    next_idx = shortest_path.route.index(vertex) + 1
                    if next_idx < len(shortest_path.route) and shortest_path.route[next_idx] != neighbor:
                        path.route.append(neighbor)
                        should_add = True
                        # print(f"{neighbor} route a eklendi")

                else:
                    path.route.append(neighbor)
                    should_add = True
                    # print(f"{neighbor} route a eklendi")

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

                if LQ[tail]:
                    heapq.heappush(global_PQ, ((not LQ[tail][0].isActive, LQ[tail][0].lb), id(LQ[tail]), LQ[tail]))
    
    print("find kspd nin ilk kısmının sonunda global pq durumu")
    print_global(global_PQ)            

    while len(result_set) < k and global_PQ:
        new_path = FindNextPath(graph, graph_state, global_PQ, LQ, threshold, result_set, dest, covered_vertices)
        
        print("find next path çıktısı olan path: ")
        print(new_path)
        
        if new_path and new_path.Sim(threshold=threshold, result_set=result_set):
            result_set.append(new_path)

    return result_set

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
    GR = reverse(G)
    
    # Run algorithm
    print("Finding top-3 shortest paths with diversity...")
    print("Source: A (1), Destination: D (4)")
    print("Threshold (τ): 0.5")
    print("=" * 60)
    
    node_names = {1: 'A', 2: 'B', 3: 'C', 4: 'D', 5: 'E', 6: 'F', 7: 'H', 8: 'I'}
    
    number_of_paths_explored = 0

    result = FindKSPD(G, GR, src=1, dest=4, k=3, threshold=0.5)

    print(f"Number of explored paths: {number_of_paths_explored}")    
    
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