from qgis.core import *
from qgis.networkanalysis import *
from qgis.utils import *

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

# Load Network
network = QgsVectorLayer("R:/RND_Projects/Project/RND073_QGIS_Toolkit/RND073_Project_Work/RND073_Axial/RND073_Existing/ae_network.shp",
                               "network",
                               "ogr")

cost_field = 'cost'
cutoff = 2000

# Root dictionary
points = []
for index, segment in enumerate(network.getFeatures()):
    points.append(segment.geometry().centroid().asPoint())

# Build graph
graph, tied_points = makeUndirectedGraph(network, cost_field, points)
i = 0
while i < 2:
    tree, cost = QgsGraphAnalyzer.dijkstra(graph, i, 0)
    s = 0
    print s
    while s < len(cost):
        if s != i: # ignore destination
            origin = s
            route = []
            while cost[origin] < 0:
                incoming_edges = graph.vertex(s).inArc()
                print incoming_edges
                min_cost_edge = min([cost[arc] for arc in incoming_edges])
                route.append(s)
                origin = min_cost_edge
                print origin
    s += 1
i += 1
# Create cost tree from network
# i = 0
# # while i < len(points):
# cost_tree = calculateCostTree(graph, tied_points, i, cutoff)
#     print i
#     i += 1