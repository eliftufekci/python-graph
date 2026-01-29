import networkx as nx
import heapq
G = nx.DiGraph()
edges = [(1, 2, 10),
         (2, 3, 1),
         (3, 4, 10),
         (1, 9, 20),
         (9, 2, 1),
         (2, 6, 1),
         (6, 8, 1),
         (8, 9, 1),
         (2, 8, 3),
         (8, 5, 15),
         (3, 5, 18),
         (5, 4, 1)]

# A 1
# B 2
# C 3
# D 4
# E 5
# F 6
# G 7
# H 8
# I 9

G.add_weighted_edges_from(edges)
nx.draw(G, with_labels=True)
def reverse(graph):
  Gr = nx.DiGraph()
  # for node in graph.nodes():
  #   Gr.add_node(node)

  Gr.add_edges_from((v,u,d) for u,v,d in graph.edges(data=True))
  return Gr
GR = reverse(G)
nx.draw(GR, with_labels=True)



class Path:
  def __init__(self):
    self.route = []
    self.edges = {}
    self.length = 0
    self.lb = 0
    self.cls = None

  def LB1(self, node, distances):
    return self.length + distances[node] 

  # result dict arrayi olarak tutuluyor
  def LB2(self, threshold, result):
    lb2 = 0

    for old_path in result: 

      old_path_edges = old_path.edges

      common_edges = set(old_path_edges.keys()).intersection(set(self.edges.keys()))

      intersection_length = sum(old_path_edges[e] for e in common_edges)

      current_lb2 = intersection_length * (1+1/threshold) - old_path.length
      lb2 = max(lb2, current_lb2)

    return lb2

  
  def Sim(self, threshold, result):
    flag = True

    for old_path in result: 
      old_path_edges = old_path.edges

      common_edges = set(old_path_edges.keys()).intersection(set(self.edges.keys()))
      intersection_length = sum(old_path_edges[e] for e in common_edges)

      # union_edges = set(new_path_edges.keys()).union(set(old_path_edges.keys()))
      # union_length = sum(new_path_edges[e] for e in union_edges if e in new_path_edges)
      # union_length += sum(old_path_edges[e] for e in union_edges if e in old_path_edges)
      # union_length -= intersection_length
      union_length = self.length + old_path.length - intersection_length

      similarity = intersection_length / union_length

      if similarity >= threshold:
        flag = False
        break  

    return flag
  
  def tail(self):
    return self.route[-1] if self.route else None

  def head(self):
    return self.route[0] if self.route else None

  def contains(self, vertex):
    return vertex in self.route


def ConstructPartialSPT(graph, v):
  global distances, isSettled, PQ, parent

  if isSettled[v]:
    return distances[v]

  while PQ:
    cost, node = heapq.heappop(PQ)

    if cost > distances[node]:
      continue

    if not isSettled[node]:
      isSettled[node] = True

      for neighbor, data in graph[node].items():

        if not isSettled[neighbor]:        
          new_cost = cost + data['weight']

          if new_cost < distances[neighbor]:
            distances[neighbor] = new_cost
            parent[neighbor] = node
            heapq.heappush(PQ, (new_cost, neighbor))

      if node == v:
        return distances[v]      
      
  return float('inf')






# path dönen dijkstra
def dijkstra(graph, src, dest):

  if src == dest:
    return {}, 0

  heap = [(0, src, [])]
  visited = set()

  while heap:
    cost, node, path = heapq.heappop(heap)

    if node in visited:
      continue
    visited.add(node)

    if node == dest:
      shortest_path = Path()

      for i in range(len(path)):
        if i < len(path) - 1:
          u, v = path[i], path[i+1]
          shortest_path.edges[(u,v)] = graph[u][v]['weight']
          shortest_path.length += graph[u][v]['weight']
          shortest_path.route.append(u)
        else:
          u, v = path[i], dest
          shortest_path.edges[(u,v)] = graph[u][v]['weight']
          shortest_path.length += graph[u][v]['weight']
          shortest_path.route.append(u)
          shortest_path.route.append(v)

      return shortest_path

    for neighbor, data, in graph[node].items():
      if neighbor not in visited:
        new_cost = cost + data['weight']
        heapq.heappush(heap, (new_cost, neighbor, path + [node]))

  return None, float('inf')





distances = {}
isSettled = {}
PQ = []
parent = {}

for node in GR.nodes():
  distances[node] = float('inf')
  isSettled[node] = False
  parent[node] = None

dest = 4 # D vertexi destination noktamız
heapq.heappush(PQ, (0, dest))
distances[dest] = 0


first_shortest_path = dijkstra(G, 1, 4)
first_shortest_path.__dict__