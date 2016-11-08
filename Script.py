from PyQt4.QtCore import *
from PyQt4.QtGui import *

from qgis.core import *
from qgis.gui import *
from qgis.networkanalysis import *

class customCost(QgsArcProperter):
    def __init__(self, costColumIndex, defaultValue):
        QgsArcProperter.__init__(self)
        self.cost_column_index = costColumIndex
        self.default_value = defaultValue

    def property(self, distance, Feature):
        cost = float(Feature.attributes()[self.cost_column_index])
        if cost <= 0.0:
            return QVariant(self.default_value)
        return cost

    def requiredAttributes(self):
        l = []
        l.append(self.cost_column_index);
        return l



def makeUndirectedGraph(network_layer, cost_field, points):
    graph = None
    tied_points = []
    if network_layer:
        network_fields = network.pendingFields()
        network_cost_index = network_fields.indexFromName(cost_field)
        director = QgsLineVectorLayerDirector(network_layer, -1, '', '', '', 3)
        properter = customCost(network_cost_index, 0)
        director.addProperter(properter)
        builder = QgsGraphBuilder(network_layer.crs())
        tied_points = director.makeGraph(builder, points)
        graph = builder.graph()
    return graph, tied_points

def calculateCostTree(graph, tied_points, origin, cutoff, impedance=0):
    cost_tree = {}
    from_point = tied_points[origin]
    from_id = graph.findVertex(from_point)

    (tree, cost) = QgsGraphAnalyzer.dijkstra(graph, from_id, impedance)

    i = 0
    while i < len(cost):
        if cost[i] <= cutoff and tree[i] != -1:
            cost_tree[i] = (graph.vertex(i), cost)
    i += 1

    return cost_tree

def calculateRouteDijkstra(graph, tied_points, origin, destination, impedance=0):
    points = []
    if tied_points:
        try:
            from_point = tied_points[origin]
            to_point = tied_points[destination]
        except:
            return points

        # analyse graph
        if graph:
            from_id = graph.findVertex(from_point)
            to_id = graph.findVertex(to_point)

            (tree, cost) = QgsGraphAnalyzer.dijkstra(graph, from_id, impedance)

            if tree[to_id] == -1:
                pass
            else:
                curPos = to_id
                while curPos != from_id:
                    points.append(graph.vertex(graph.arc(tree[curPos]).inVertex()).point())
                    curPos = graph.arc(tree[curPos]).outVertex()

                points.append(from_point)
                points.reverse()

    return points

def calculateTreeDijkstra(graph, tied_points, origin, radius, results, impedance=0):
    if tied_points:
        to_point = tied_points[origin]

    # analyse graph
    if graph:
        to_id = graph.findVertex(to_point)

        (tree, cost) = QgsGraphAnalyzer.dijkstra(graph, to_id, impedance)

        for edge in range(len(cost)):
            if cost[edge] > radius and tree[edge] == -1:
                pass
            else:
                vertex_id = graph.arc(edge).inVertex()
                print vertex_id

                while cost[vertex_id] > 0:
                    results[vertex_id] += 1

                    outgoing_costs = [cost[graph.arc(outgoing_edge).inVertex()] for outgoing_edge in outgoing_edge_ids]
                    min_cost_index = outgoing_costs.index(min(outgoing_costs))
                    vertex_id = graph.arc(outgoing_edge_ids[min_cost_index]).inVertex()

    return results

# # Load Network
# network = QgsVectorLayer("R:/RND_Projects/Project/RND073_QGIS_Toolkit/RND073_Project_Work/RND073_Axial/RND073_Existing/ae_network.shp",
#                                "network",
#                                "ogr")

network = QgsVectorLayer("/Users/laurensversluis/Google Drive/Utopia_Cureton_Versluis/Ax_Ex_P/Ax_Ex_P.shp","network", "ogr")

cost_field = 'cost'
cutoff = 50000

# Get nodes
points = []
for index, segment in enumerate(network.getFeatures()):
    points.extend(segment.geometry().asPolyline())
unique_points = list(set(points))

# Build graph
graph, tied_points = makeUndirectedGraph(network, cost_field, unique_points)

# Result dictionary
graph_dict = {graph.findVertex(tied_point): {'neighbours': [], 'path_count': 0} for tied_point in tied_points}

# Update neighbours
for edge in graph.edgeCount:
    # get vertices
    # get neighbouring arcs
    # get neighbouring vertices
    # update graph_dict

# Update path count
for origin in range(10):
    print origin
    results = calculateTreeDijkstra(graph, tied_points, origin, cutoff, results)


print 'finito!'




#     s += 1
# i += 1
# # Create cost tree from network
# i = 0
# # while i < len(points):
# cost_tree = calculateCostTree(graph, tied_points, i, cutoff)
#     print i
#     i += 1

# i = 0
# while i < 2:
#     tree, cost = QgsGraphAnalyzer.dijkstra(graph, i, 0)
#     s = 0
#     print s
#     while s < len(cost):
#         if s != i: # ignore destination
#             origin = s
#             route = []
#             while cost[origin] < 0:
#                 incoming_edges = graph.vertex(s).inArc()
#                 print incoming_edges
#                 min_cost_edge = min([cost[arc] for arc in incoming_edges])
#                 route.append(s)
#                 origin = min_cost_edge
#                 print origin