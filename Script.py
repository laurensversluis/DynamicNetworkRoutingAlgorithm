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

def calculateTreeDijkstra(graph, to_node_id, radius, neighbours, results):

    (tree, cost) = QgsGraphAnalyzer.dijkstra(graph, to_node_id, 0)

    for edge in range(len(cost)):
        if cost[edge] > radius and tree[edge] == -1:
            pass
        else:
            vertex_id = graph.arc(edge).inVertex()

            while cost[vertex_id] > 0:
                # Determine neighbouring edge with minimum cost
                connected_vertex_ids = neighbours[vertex_id]
                cost_list = [cost[vertex_id] for vertex_id in connected_vertex_ids]
                min_cost_index = cost_list.index(min(cost_list))
                vertex_id = connected_vertex_ids[min_cost_index]
                # Write result
                results[vertex_id] += 1




# # Load Network
# network = QgsVectorLayer("R:/RND_Projects/Project/RND073_QGIS_Toolkit/RND073_Project_Work/RND073_Axial/RND073_Existing/ae_network.shp",
#                                "network",
#                                "ogr")

network = QgsVectorLayer("/Users/laurensversluis/Google Drive/GEO-Server/Central_London.shp","network", "ogr")

cost_field = 'cost'
cutoff = 500

# Get nodes
points = []
for index, segment in enumerate(network.getFeatures()):
    points.extend(segment.geometry().asPolyline())
unique_points = list(set(points))

# Build graph
graph, tied_points = makeUndirectedGraph(network, cost_field, unique_points)

# Result dictionary
results = {graph.findVertex(tied_point): 0 for tied_point in tied_points}
neighbours = {graph.findVertex(tied_point): [] for tied_point in tied_points}


# Update neighbours
for vertex_id in range(len(tied_points)):
    # get vertices
    incoming_vertex_ids = [graph.arc(edge_id).inVertex() for edge_id in graph.vertex(vertex_id).outArc()]
    outgoing_vertex_ids = [graph.arc(edge_id).outVertex() for edge_id in graph.vertex(vertex_id).inArc()]
    connected_vertex_ids = incoming_vertex_ids + outgoing_vertex_ids

    # update results
    neighbours[vertex_id] = connected_vertex_ids

# Update path count
for origin in range(10):
    print origin
    to_point = tied_points[origin]
    to_point_id = graph.findVertex(to_point)
    calculateTreeDijkstra(graph, to_point_id, cutoff, neighbours, results)

print 'finito!'