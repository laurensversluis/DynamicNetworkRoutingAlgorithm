from graph_tool.all import *
import networkx
import qgis.core

# Create graph
g = Graph()

# Loop through network get nodes
layer = iface.activeLayer()
features = layer.getFeatures()
nodes_list = []
nodes_dict = {}

# Create list of nodes
for f in features:
    start = f.geometry().asPolyline()[0].toString()
    end = f.geometry().asPolyline()[-1].toString()
    nodes_list.extend([start,end])

# Add nodes to graph and dictionary
for index, node in enumerate(set(nodes_list)):
    vertex = g.add_vertex(index)
    nodes_dict[index] = node

# Add edges to graph
for f in features:
    start_point = f.geometry().asPolyline()[0].toString()
    end_point = f.geometry().asPolyline()[-1].toString()

    start_vertex = nodes_dict.get(start_point)
    end_vertex = nodes_dict.get(end_point)

    e = g.add_edge(start_vertex,end_vertex)

vp, ep = graph_tool.centrality.betweenness(g)







