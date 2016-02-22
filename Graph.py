#!/usr/bin/python
# -*- coding: latin-1 -*-
"""
Python Library implementing basic graph manipulation and traversal
"""

from Queue import Queue

class Node(object):
	"""Node class to store attributes and neighbors"""
	def __init__(self, id, attributes = None):
		self.id = str(id)
		if attributes is None: attributes = {}
		self.attributes = attributes
		self.neighbors = []  # list of neighbors id (ajacency list implementation)

	def __str__(self):
		"""Simply prints node content"""
		return "  id: %s, attributes: %s, neighbors: %s" % (self.id, self.attributes, ','.join(self.neighbors))

class Edge(object):
	"""Edge class to store edge attributes"""
	def __init__(self, source, destination, attributes = None):
		self.source      = source
		self.destination = destination
		if attributes is None: attributes = {}
		self.attributes  = attributes

	def __str__(self):
		"""Simply prints edge"""
		return "  %s --- %s , attributes: %s" % (self.source.id, self.destination.id, self.attributes)

class Graph(object):
	"""
	Abstract class representing a graph. It stores attributes for the graph, a dictionnary of nodes by id and the edges as a list.
	"""
	# constructor
	def __init__(self):
		self.attributes = {}
		self.nodesById  = {}
		self.edges      = []

	def __str__(self):
		"""Dumps all nodes and edges of the graph."""
		s = 'Attributes: %s \n\nNodes (%s)\n' % (self.attributes, len(self.nodesById.keys()))
		for n in self.nodes(): s+= str(n)+'\n'
		s+= '\nEdges (%s)\n' % (len(self.edges))
		for e in sorted(self.edges, key=lambda edge: edge.source.id): s+= str(e)+'\n'
		s+= '\n'
		return s


	# modifiers
	###########
	def addNode(self, node, attributes = None):
		"""
		Add a node to the graph.
		It can be called with an already existing node:
			n = Node('youpi')
			g.addNode(n)
		or it can be called with an id:
			g.addNode('youpi')
		Attributes can be passed through a dictionnary:
			g.addNode('yopla', { 'color': 'yellow', 'flavour': 'lemon' })
		"""
		if isinstance(node, Node): #  node variable is a Node object
			if node.id in self.nodesById:
				return self.nodesById[node.id] # return it if already in this graph
			# copy attributes if any were passed
			if attributes is not None:
				for a in attributes:
					node.attributes[a] = attributes[a]
			elif node.attributes is None: # or initialize if null
				node.attributes = {}
			# add it to our dict
			self.nodesById[node.id] = node
			return node # return new node object
		else: # not a node instance, assume it is an id, need to create one
			if node in self.nodesById: # return it if already in this graph
				return self.nodesById[node]
			# initialize attributes if needed
			if attributes is None:
				attributes = {}
			# create node
			n = Node(id = node, attributes=attributes)
			self.nodesById[ n.id ] = n
			return n # return new node object



	# accessors
	###########
	def node(self, obj): # convert node id or node to node (to ensure we have a node)
		"""
		Utility function to ensure we have a node:
			n = g.node(n)
		if n is a Node instance then it does nothing, but if n is the node id then it is replaced by the Node instance.
		"""
		if isinstance(obj, Node):
			return obj
		else:
			for n in self.nodesById.values():
				if n.id == obj: return n
			return None


	def nodes(self): # get all nodes as a vector
		"""
		Return the set of nodes of the graph as a list sorted lexicographically by their id.
		"""
		return sorted(self.nodesById.values(), key=lambda node: node.id)


	def neighbors(self, node):
		"""
		Returns the set of Nodes (as a list) that are accessible from node.
		"""
		if isinstance(node, Node) == False:
			 n = self.node(node)
		else: n = node
		
		neighbors = []
		if isinstance(n, Node) == False: return None
		for nei in n.neighbors:
			neighbors.append(self.nodesById[nei])
		return neighbors


	def isAcyclic(self):
		"""
		Tests if the graph is devoid of any cycling paths.
		Calls dfs() and derives decision from its outcome.
		Nodes gain attributes given by dfs()
		"""
		self.dfs()
		for edge in self.edges:
			if edge.attributes["dfs_type"] == "back":
				return False
		return True


	def adjacencyMatrix(self, weight = None):
		for n in self.nodesById.values():
			if weight not in n.attributes.keys():
				print 'Attribute error: ' + weight + " parameter doesn't match any attribute in at least one of the graph nodes"
				return None
		matrix = {}
		for i in self.nodesById:
			matrix[i] = {}
			for j in self.nodesById:
				matrix[i][j] = float('inf')
				matrix[i][i] = 0.0
				if self.edge(i,j) != None:
					if weight != None: matrix[i][j] = float(self.edge(i,j).attributes[weight])
					else: matrix[i][j] = 1.0
		return matrix



	# traversal
	###########

	## Depth First Search travserval:
	def dfs(self):
		"""
		Depth First Search travserval of the graph.
		Makes call to dfsVisit(node).
		After execution, nodes have additional attributes:
		  - dfs_color: should be black (node has been processed)
		  - dfs_predecessor: node id of predecessor in the traversal
		  - dfs_in: time when the node started to be processed
		  - dfs_out: time when the node processing finished
		edges have an additional attribute:
		  - dfs_type: dfs classification of edges
		     tree: the edge is part of a dfs tree
		     back: the edge creates a cycle
		     forward: from a node to a reachable node in the dfs tree
		     cross: from a node to a node from an other dfs tree
		"""
		for node in self.nodesById.values():
			node.attributes["dfs_color"] = "WHITE"
			node.attributes["dfs_predecessor"] = None
		self.attributes["dfs_time"] = 0
		for node in self.nodes():
			if node.attributes["dfs_color"] == "WHITE":
				self.dfsVisit(node)
		return None

	def dfsVisit(self, node):
		node.attributes["dfs_color"] = "GREY"
		self.attributes["dfs_time"] += 1
		node.attributes["dfs_in"] = self.attributes["dfs_time"]
		for nei in self.neighbors(node):
			if nei.attributes["dfs_color"] == "WHITE":
				nei.attributes["dfs_predecessor"] = node.id
				self.dfsVisit(nei)
				self.edge(node, nei).attributes["dfs_type"] = "tree"
			elif nei.attributes["dfs_color"] == "GREY":
				self.edge(node, nei).attributes["dfs_type"] = "back"
			elif node.attributes["dfs_in"] > nei.attributes["dfs_in"]:
				self.edge(node, nei).attributes["dfs_type"] = "cross"
			else: self.edge(node, nei).attributes["dfs_type"] = "forward"
		node.attributes["dfs_color"] = "BLACK"
		self.attributes["dfs_time"] += 1
		node.attributes["dfs_out"] = self.attributes["dfs_time"]
		return None
	


	## Breadth First Search traverval:
	def bfs(self,source):
		"""
		Breadth First Search travserval of the graph.
		After execution, nodes have additional attributes:
		  - bfs_color: At the end: black is successor of node, white if not (predecessor of node or its successors)
		  - bfs_predecessor: node id of predecessor in the traversal.
		  - bfs_in: time when the node started to be processed
		"""
		source = self.node(source)
		if source == None:
			print 'Innexistant Node: Node given as source not registered as part of this graph.'
			return None
		for n in self.nodesById.values():
			n.attributes["bfs_color"] = "WHITE"
			n.attributes["bfs_in"] = float("inf")
			n.attributes["bfs_predecessor"] = None
		source.attributes["bfs_color"] = "GREY"
		source.attributes["bfs_in"] = 0
		Q = Queue(maxsize=0)
		Q.put(source)
		while Q.empty() == False:
			n = Q.get()
			for nei in self.neighbors(n):
				if nei.attributes["bfs_color"] == "WHITE":
					nei.attributes["bfs_color"] = "GREY"
					nei.attributes["bfs_in"] = n.attributes["bfs_in"] +1
					nei.attributes["bfs_predecessor"] = n.id
					Q.put(nei)
			n.attributes["bfs_color"] = "BLACK"
		return None



	## Shortest Path Bellman-Ford travserval:

	def bellman_ford(self, source, weight):
		"""
		Bellman-Ford single source shortest path traversal of the graph.
		After execution, nodes have additional attributes:
		  - BF_d<-src: Distance(or cost) of the shortest path from source to the node
		  - bfs_predecessor: node id of predecessor in the shortest path from source to the node
		"""
		for n in self.nodesById.values():
			if weight not in n.attributes.keys():
				print 'Attribute error: ' + weight + " parameter doesn't match any attribute in at least one of the graph nodes"
				return None
		source = self.node(source)
		if source == None:
			print 'Innexistant Node: Node given as source not registered as part of this graph.'
			return None
		self.BF_initializeSingleSource(source)
		for i in xrange(1, len(self.edges)-1): 
			for e in self.edges:
				self.BF_relax(e.source, e.destination, float(e.attributes[weight]))

	def BF_initializeSingleSource(self, source):
		for n in self.nodesById.values():
			n.attributes["BF_d<-src"] = float("inf")
			n.attributes["BF_path_pred"] = None
		source.attributes["BF_d<-src"] = 0
		return None

	def BF_relax(self, source, destination, weight):
		if destination.attributes["BF_d<-src"] > source.attributes["BF_d<-src"] + weight:
			destination.attributes["BF_d<-src"] = source.attributes["BF_d<-src"] + weight
			destination.attributes["BF_path_pred"] = source.id



	## Shortest Paths Floyd-Warshall traversal:
	def floyd_warshall(self, weight):
		for n in self.nodesById.values():
			if weight not in n.attributes.keys():
				print 'Attribute error: ' + weight + " parameter doesn't match any attribute in at least one of the graph nodes"
				return None
		# Matrices initialization:
		D = self.adjacencyMatrix(weight)
		N = {}
		for n1 in D.keys():
			N[n1] = {}
			for n2 in D.keys():
				N[n1][n2] = None
		for e in self.edges:
			N[e.source.id][e.destination.id] = e.destination.id
		for i in D.keys():
			for j in D.keys():
				for k in D.keys():
					if D[i][j] + D[k][j] < D[i][j]:
						D[i][j] = D[i][k] + D[k][j]
						N[i][j] = N[i][k]
		return [D, N]

	def FW_shortest_path(self, D, N, i, j):
		if isinstance(i, Node): i = i.id
		if isinstance(j, Node): i = j.id
		if self.node(i) == None or self.node(j) == None: print "No such node(s)!"
		if D[i][j] == float('inf'):
			return ['No path from '+ i + ' to ' + j]
		path = [i]
		k = N[i][j]
		while k != None:
			path.append(k)
			k = N[k][j]
		path.append(j)
		return path

	# File Loaders:
	###############
	def loadSIF(self, filename):
		"""
		Loads a graph in a (simplified) SIF (Simple Interactipon Format, cf. Cytoscape doc).
		Assumed input:
		   node1	relation	node2
		   chaussettes	avant	chaussures
		   pantalon	avant	chaussures
		   ...
		"""
		with open(filename) as f:
			# SKIP COLUMNS NAMES
			tmp = f.readline()
			# PROCESS THE REMAINING LINES
			row = f.readline().rstrip()
			while row:
				vals = row.split('\t')
				self.addEdge(vals[0], vals[2])
				row = f.readline().rstrip()
		return None

	def loadTAB(self, filename):
		"""
		Loads a graph in Cytoscape tab format
		Assumed input:
		   id1	id2	weight	color	...
		   A	B	6	blue	...
		"""
		with open(filename) as f: 
			# GET COLUMNS NAMES
			tmp = f.readline().rstrip()
			attNames= tmp.split('\t')
			# REMOVES FIRST TWO COLUMNS WHICH CORRESPONDS TO THE LABELS OF THE CONNECTED VERTICES
			attNames.pop(0)
			attNames.pop(0)
			# PROCESS THE REMAINING LINES
			row = f.readline().rstrip()
			while row:
				vals = row.split('\t')
				v1 = vals.pop(0)
				v2 = vals.pop(0)
				att = {}
				for i in xrange(len(attNames)):
					att[ attNames[i] ] = vals[i]
				self.addEdge(v1, v2, att)
				row = f.readline().rstrip() # NEXT LINE


class DirectedGraph(Graph):
	"""
	Class implementing a directed graph (subclasses Graph) as adjacency lists.
	"""
	# constructor
	def __init__(self):
		super(DirectedGraph, self).__init__()

	# modifiers
	###########
	def addEdge(self, source, destination, attributes = None):
		"""
		Add an edge to the graph.
		The source and destination can be either nodes or node ids.
		If the nodes do not exist, they will be created and added to the graph:
		   g=DirectedGraph()
		   g.addEdge('chaussettes','chaussures')
		is equivalent to:
		   src=Node('chaussettes')
		   dst=Node('chaussures')
		   g.addEdge(src,dst)
		Attributes can be set on edges:
		   g.addEdge('Toulouse', 'Bordeaux', { 'dist' : 250 } )
		"""
		s = self.addNode(source) # Adds node if necessary as defined in addNode.
		d = self.addNode(destination)
		#Test if edge is not already in the graph.
		for e in self.edges:
			if e.source.id == s.id and e.destination.id == d.id:
				return e #Edge exists. Return it.
		#Else add a new edge to the graph.
		s.neighbors.append(d.id)
		edge = Edge(s,d,attributes)
		self.edges.append(edge)
		return edge



	# accessors
	###########
	def isDirected(self):
		"""
		returns True!
		"""
		return True

	def edge(self, source, destination):
		"""
		Used to ensure we have an edge from source to destination, and retrieve attributes:
		  e = g.edge('Toulouse', 'Bordeaux')
		  print e.attributes['dist']
		"""
		source = self.node(source)
		destination = self.node(destination)

		for e in self.edges:
			if e.source == source and e.destination == destination: return e
		return None

	# traversal
	###########
	def topologicalSort(self):
		"""
		Performs a topological sort of the nodes of the graph. Nodes are return as a list.
		Absence of cycle is not tested.
		"""
		if self.isAcyclic() == False: 
			return None
		self.dfs()
		nodes = self.nodes()
		print type (nodes)
		nodes.sort(key=lambda node: node.attributes["dfs_out"], reverse = True)
		return nodes

# TESTS
if __name__ == "__main__":
	#~ print "TP1"
	#~ print "Testing Graph.loadSIF:"
	#~ g = DirectedGraph()
	#~ g.loadSIF('dressing.sif')
	#~ g.loadSIF('dia29.sif')
	#~ g.loadSIF('String_EcolA_experimental.sif')
	#~ print "Testing Graph.loadTAB:"
	#~ g.loadTAB('Bellman-Ford.tab')
	#~ g.loadTAB('Floyd-Warshall.tab')
	#~ print g
	#~ ###
	#~ print "Testing accessors:"
	#~ print g.node('ceinture')
	#~ print g.node('veste')
	#~ print g.edge('ceinture', 'veste')
	#~ print g.edge('veste', 'ceinture')
	#~ print g.neighbors('ceinture')
	#~ g.adjacencyMatrix('weight')
	#~ ###
	#~ print "Testing DFS"
	#~ g.dfs()
	#~ print g
	#~ ###
	#~ print "Testing BFS"
	#~ g.bfs('sous-vetements')
	#~ print g
	#~ ###
	#~ print "Testing Bellman-Ford"
	#~ g.bellman_ford('A', 'potato')
	#~ print g
	#~ print "Testing Floyd-Warshall"
	#~ g.floyd_warshall('weight')
	#~ print g.FW_shortest_path(g.floyd_warshall('weight')[0], g.floyd_warshall('weight')[1], 'A', 'B')
	#~ print g.FW_shortest_path(g.floyd_warshall('weight')[0], g.floyd_warshall('weight')[1], 'A', 'C')
	#~ ###
	#~ print "The graph is acyclic? " + str(g.isAcyclic())
	#~ print "Topological sort:"
	#~ nodes = g.topologicalSort()
	#~ for n in nodes: print "%s (%i) " % (n.id , n.attributes['dfs_out'] )
