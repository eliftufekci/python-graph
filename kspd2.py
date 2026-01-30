import networkx as nx
import heapq

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
		return f"Route: {self.route}, Length: {self.length}, LB: {self.lb}, Class: {self.cls}"
	
	def __lt__(self, other):
		return self.lb < other.lb 
	
	def tail(self):
		return self.route[-1] if self.route else None

	def head(self):
		return self.route[0] if self.route else None

	def contains(self, vertex):
		return vertex in self.route
	
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
			ConstructPartialSPT(graph_state, tail) 

		return self.lenght + graph_state.distances[tail]

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


def reverse(graph):
	Gr = nx.DiGraph()
	Gr.add_edges_from((v,u,d) for u,v,d in graph.edges(data=True))
	return Gr


def dijkstra(graph, src, dest):
	if src == dest
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

*** değişecek
def ExtendPath(path: Path, LQ, graph, global_PQ):
    global distances
    tail = path.tail()

    for p in LQ[tail]:
        if p.cls == path.cls and p.length >= path.length:
            p.isActive = False
    
    for neighbor, data in graph[tail].items():
        if neighbor not in path.route and neighbor != tail.parent:
            new_path = Path()
            new_path.route = path.route + [neighbor]
            new_path.edges = path.edges.copy().update({(tail, neighbor): data['weight']})
            new_path.length = path.length + data['weight']
            new_path.cls = path.cls
            new_path.lb = new_path.LB1(neighbor, distances)

            if neighbor has been covered by path.cls:
                new_path.isActive = False
            else:
                heapq.heappush(LQ[neighbor], new_path)
                if LQ[neighbor] not in global_PQ:
                    heapq.heappush(global_PQ, (LQ[neighbor][0].lb, LQ[neighbor]))

    if tail.parent in path.route: 
        return False
    else:
        path.route.append(tail.parent)
        weight = graph[tail][tail.parent]['weight']
        path.length += weight
        path.edges[(tail, tail.parent)] = weight
        return True

def FindKSPD(graph, src, dest, k, threshold):
	global result_set, distances

	shortest_path = dijkstra(graph, src, dest)
	result_set.append(shortest_path)

	global_PQ = []
	LQ = {}

	for vertex in shortest_path.route[:-1]:
		# print("----")
		# print(f"vertex: {vertex}")

		for neighbor in graph[vertex]:
			path = Path()
			path.route = shortest_path.route[:shortest_path.route.index(vertex)+1]
		#     print(f"path route: {path.route}")

			flag = False
		#     print(f"neighbor: {neighbor}")

			if not path.contains(neighbor):
				if shortest_path.contains(neighbor):
					if shortest_path.route.index(vertex)+1 != shortest_path.route.index(neighbor) :
						path.route.append(neighbor)
						flag = True
						# print(f"{neighbor} route a eklendi")
					else:
						continue

				else:
					path.route.append(neighbor)
					flag = True
				#     print(f"{neighbor} route a eklendi")
			
			if flag:
				for i in range(len(path.route)-1):
					u, v = path.route[i], path.route[i+1]
					path.edges[(u,v)] = graph[u][v]['weight']
					path.length += graph[u][v]['weight']
				path.cls = (1, vertex)
				path.lb = path.LB1(vertex, distances)

				tail = path.tail()
				LQ[tail] = []
				heapq.heappush(LQ[tail], path)

				if LQ[tail] not in global_PQ:
					heapq.heappush(global_PQ, (LQ[tail][0].lb, LQ[tail]))

		# for a in global_PQ:
		#     print(a)

	while len(result_set) < k and global_PQ:
		# current_path = heapq.heappop(global_PQ)

		# if current_path.tail() == dest:
		#     if current_path.Sim(threshold, result_set):
		#         result_set.append(current_path)

		# else:
		#     extended_paths = ExtendPath(current_path)

		# for path in extended_paths:
		#     heapq.heappush(global_PQ, path)

		new_path = FindNextPath(graph, global_PQ, threshold, src, dest, LQ)
		if new_path.Sim(threshold, result_set):
			result_set.append(new_path)

	return result_set
 
FindKSPD(G, 1, 4, 3, 0.5)
def FindNextPath(graph, global_PQ, threshold, src, dest, LQ):
	while global_PQ:

		_, Q = heapq.heappop(global_PQ)
		current_path = heapq.heappop(Q)

		if Q:
			heapq.heappush(global_PQ, (Q[0].lb, Q))
		
		while current_path.tail() != dest:
			LB2 = current_path.LB2(threshold, result_set)
			
			if LB2 > current_path.lb:
				current_path.lb = LB2
				AdjustPath(current_path, LQ, src, dest)

				heapq.heappush(Q, current_path)
				if Q not in global_PQ:
					heapq.heappush(global_PQ, (Q[0].lb, Q))
				break
			
			elif not ExtendPath(current_path):
				break

		if current_path.tail() == dest:
			AdjustPath(current_path, LQ, src, dest)
			return current_path

def AdjustPath(path: Path, LQ, src, dest):
	_, deviation_vertex = path.cls

	for vertex in path.route[deviation_vertex:]:
		for p in LQ[vertex]:
			if not p.isActive:
				p.isActive = True
			
			if p.tail == dest:
				for local_queue in LQ.values():
					if vertex in local_queue.route:
						local_queue.cls = (len(result_set),vertex)

