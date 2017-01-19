import networkx as nx
from PyQt4.QtCore import QVariant

vectorlayer = iface.activeLayer()

def constructPrimalGraph(vectorlayer, custom_cost_field):
    # Create undirected empty graph
    G = nx.Graph()

    # Read segments and build dualgraph
    for segment in vectorlayer.getFeatures():
        # Calculating properties
        geom = segment.geometry()
        pt1 = QgsPoint(geom.asPolyline()[0])
        pt2 = QgsPoint(geom.asPolyline()[1])
        azimuth = pt1.azimuth(pt2)
        metric_cost = geom.length()
        custom_cost = segment[custom_cost_field]
        # Determine primal nodes
        G.add_edge(pt1, pt2, azimuth=azimuth, metric_cost=metric_cost, custom_cost=custom_cost)

    return G

def constructDualGraph(primalGraph):
    # Create undirected empty graph
    G = nx.Graph()
    # Read edges and add as nodes to graph
    for primal_node1, primal_node2, azimuth in primalGraph.edges(data='azimuth'):
        primal_edge1 = (primal_node1, primal_node2)
        dual_node1 = QgsGeometry.fromPolyline([QgsPoint(primal_node1), QgsPoint(primal_node2)]).centroid()
        # Construction of all connections to start node
        start_neighbors = primalGraph.neighbors(primal_node1)
        for node in start_neighbors:
            if not node == primal_node2:
                edge_azimuth = primalGraph[primal_node1][node]['azimuth']
                # Calculating the angle between two edges
                angle = 360 - abs(azimuth - edge_azimuth)
                if angle > 180: # wide angles
                    angleCost = 180 - (360 - abs(angle))
                else: # acute angles
                    angleCost = 180 - angle
                primal_edge2 = (primal_node1, node) # target node
                # Construction addition of the dualGraph edge
                dual_node2 = QgsGeometry.fromPolyline([QgsPoint(primal_node1), QgsPoint(node)]).centroid()
                dual_edge = QgsGeometry.fromPolyline([dual_node1, dual_node2])
                G.add_edge(primal_edge1, primal_edge2, geom=dual_edge, angleCost=angleCost)
        # Construction of all connections to end node
        end_neigbors = primalGraph.neighbors(primal_node2)
        for node in end_neigbors:
            if not node == primal_node1:
                edge_azimuth = primalGraph[primal_node2][node]['azimuth']
                # Calculating the angle between two edges
                angle = 360 - abs(azimuth - edge_azimuth)
                if angle > 180: # wide angles
                    angleCost = 180 - (360 - abs(angle))
                else: # acute angles
                    angleCost = 180 - angle

                primal_edge2 = (primal_node2, node) # target node
                # Construction addition of the dualGraph edge
                dual_node2 = QgsGeometry.fromPolyline([QgsPoint(primal_node1), QgsPoint(node)]).centroid()
                dual_edge = QgsGeometry.fromPolyline([dual_node1, dual_node2])
                G.add_edge(primal_edge1, primal_edge2, geom=dual_edge, angleCost=angleCost)

    return G

def writeGraph(graph):
    # create layer
    vl = QgsVectorLayer("Line", "graph", "memory")
    pr = vl.dataProvider()

    for node1, node2, edge_geom in graph.edges(data='geom'):
        # add a feature
        fet = QgsFeature()
        fet.setGeometry(edge_geom)
        pr.addFeatures([fet])

    # update layer's extent when new features have been added
    # because change of extent in provider is not propagated to the layer
    vl.updateExtents()

G = constructPrimalGraph(vectorlayer, 'id')
DG = constructDualGraph(G)
writeGraph(DG)
