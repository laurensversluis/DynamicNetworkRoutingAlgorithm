# Dependencies
import networkx as nx

# Add edges and nodes based on dual graph type
def constructPrimalGraph(vectorlayer, custom_cost_field):
    # Create empty graph
    G = nx.Graph()

    # Read segments and build dualgraph
    for segment in vectorlayer.getFeatures():
        # Calculating properties
        geom = segment.geometry()
        pt1 = QgsPoint(geom.asPolyline()[0])
        pt2 = QgsPoint(geom.asPolyline()[1])
        azimuth = pt1.azimuth(pt2)
        length = geom.length()
        custom_cost = segment[custom_cost_field]
        # Determine primal nodes
        G.add_edge(pt1, pt2, azimuth=azimuth, metric_cost=length, custom_cost=custom_cost)

    return G

def constructDualGraph(primalGraph):
    # Create empty graph
    G = nx.Graph()
    # Read edges and add as nodes to graph
    for start_node, end_node, azimuth in primalGraph.edges(data='azimuth'):
        #
        start_neighbors = primalGraph.neighbors(start_node)
        end_neigbors = primalGraph.neighbors(end_node)
        neighbors = [start_neighbors, end_neigbors]
        for node in start_neighbors:
            if not node = end_node:
                edge = (start_node, node)
                edge_azimuth = primalGraph[start_node][end_node]['azimuth']
                # Calculating the angle between two edges
                angle = 360 - abs(azimuth - edge_azimuth)
                if angle > 180: # wide angles
                    angleCost = 180 - (360 - abs(angle))
                else: # acute angles
                    angleCost = 180 - angle

        end_neigbors = primalGraph.neighbors(end_node)
        G.add_node()

    return G

    g.addEdge(1,2, angular=0.5, metric=0.5)
    g.addEdge(1,2, angular=0.2, metric=0.5)
    g.addEdge(1,3, angular=0.8, metric=0.5)
    g.addEdge(2,4, angular=0.2, metric=0.5)
    g.addEdge(3,4, angular=0.6, metric=0.5)
    g.addEdge(4,5, angular=0.2, metric=0.5)


# Cost formula
cost_formula = {angular: 0.8, metric: 0.2}

# Route from source until target is reached
def routeGraph(source, target, cost_formula):
    path = [source, ] # Travelled edges
    while node != target:
        connected_nodes = g.neighbors(node)
        min_distance = None
        new_node = None
        for node in connected_nodes:
            if not node in path: # No turning back
                for cost, pct in cost_formula
                    distance = nx.shortest_path_length(g, node, target, cost) * pct
                    if distance < min_distance or min_distance = None:
                        new_node = node

        path.append(new_node)
        node = new_node

    return path

path = routePath(1,5)

# Creating edges
dual_node1 = QgsGeometry.fromPolyline([QgsPoint(pn1), QgsPoint(pn2)]).centroid().asPoint()
# Construction of all connections to start node
start_neighbors = primalGraph.neighbors(pn1)
for node in start_neighbors:
    if not node == pn2:
        edge_azimuth = primalGraph[pn1][node]['azimuth']
        # Calculating the angle between two edges
        angle = 360 - abs(azimuth - edge_azimuth)
        if angle > 180:  # wide angles
            angleCost = 180 - (360 - abs(angle))
        else:  # acute angles
            angleCost = 180 - angle
        primal_edge2 = (pn1, node)  # target node
        # Construction addition of the dualGraph edge
        dual_node2 = QgsGeometry.fromPolyline([QgsPoint(pn1), QgsPoint(node)]).centroid().asPoint()
        dual_edge = QgsGeometry.fromPolyline([dual_node1, dual_node2])
        G.add_edge(primal_edge1, primal_edge2, geom=dual_edge, angleCost=angleCost)
# Construction of all connections to end node
end_neigbors = primalGraph.neighbors(pn2)
for node in end_neigbors:
    if not node == pn1:
        edge_azimuth = primalGraph[pn2][node]['azimuth']
        # Calculating the angle between two edges
        angle = 360 - abs(azimuth - edge_azimuth)
        if angle > 180:  # wide angles
            angleCost = 180 - (360 - abs(angle))
        else:  # acute angles
            angleCost = 180 - angle

        primal_edge2 = (pn2, node)  # target node
        # Construction addition of the dualGraph edge
        dual_node2 = QgsGeometry.fromPolyline([QgsPoint(pn1), QgsPoint(node)]).centroid().asPoint()
        dual_edge = QgsGeometry.fromPolyline([dual_node1, dual_node2])
        G.add_edge(primal_edge1, primal_edge2, geom=dual_edge, angleCost=angleCost)