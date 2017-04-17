import math
import networkx as nx
from PyQt4.QtCore import QVariant

vectorlayer = iface.activeLayer()


def getAngle(x1, x2, y1, y2):
    inner_product = x1 * x2 + y1 * y2
    len1 = math.hypot(x1, y1)
    len2 = math.hypot(x2, y2)
    try:
        angle = math.acos(inner_product / (len1 * len2))
        ang_deg = math.degrees(angle) % 360
        if ang_deg - 180 >= 0:
            # As in if statement
            return 360 - ang_deg
        else:

            return ang_deg
    except:
        return None

def constructPrimalGraph(vectorlayer, custom_cost_field):
    # Create undirected empty graph
    G = nx.Graph()

    # Read segments and build dualgraph
    for segment in vectorlayer.getFeatures():
        # Calculating properties
        geom = segment.geometry()
        pn1 = geom.asPolyline()[0]
        pn2 = geom.asPolyline()[1]
        metric_cost = geom.length()
        custom_cost = segment[custom_cost_field]

        # Determine primal nodes
        G.add_edge(pn1, pn2, geom=geom.asPolyline(), metric_cost=metric_cost, custom_cost=custom_cost)

    return G

def constructDualGraph(primalGraph):
    # Create undirected empty graph
    G = nx.Graph()

    # Create dual graph nodes
    for pn1, pn2, data in primalGraph.edges_iter(data=True):

        # Node geometry based on centroid
        dn_geom = QgsGeometry.fromPolyline([QgsPoint(pn1), QgsPoint(pn2)]).centroid()

        # Creating graph node
        dn = dn_geom.asPoint()
        G.add_node(dn,
                   geom=dn_geom,
                   p_geom=QgsGeometry.fromPolyline([QgsPoint(pn1), QgsPoint(pn2)]),
                   metric_cost=data['metric_cost'],
                   custom_cost=data['custom_cost'])

    # Creating dual graph edges
    for pn1, data in primalGraph.nodes_iter(data=True):

        # Find neighboring dual graph nodes
        neighbors = [pn2 for pn2 in primalGraph[pn1] if pn2 != pn1]

        # Getting the first neighbor and getting its dual node
        for pn2 in neighbors:
            dn1 = QgsGeometry.fromPolyline([QgsPoint(pn1), QgsPoint(pn2)]).centroid().asPoint()
            dn1_geom = G.node[dn1]['geom']
            dn1_x = pn2.x() - pn1.x()
            dn1_y = pn2.y() - pn1.y()
            dn1_metric_cost = G.node[dn1]['metric_cost']
            dn1_custom_cost =G.node[dn1]['custom_cost']


            # Getting the second neighbor and getting its dual node
            for pn3 in neighbors:
                if pn3 != pn2:
                    dn2 = QgsGeometry.fromPolyline([QgsPoint(pn1), QgsPoint(pn3)]).centroid().asPoint()
                    dn2_geom = G.node[dn2]['geom']
                    dn2_x = pn3.x() - pn1.x()
                    dn2_y = pn3.y() - pn1.y()
                    dn2_metric_cost = G.node[dn2]['metric_cost']
                    dn2_custom_cost = G.node[dn2]['custom_cost']

                    # Calculating the angular cost between the dual nodes / primal edges
                    de_angle_cost = getAngle(dn1_x, dn2_x, dn1_y, dn2_y)
                    if de_angle_cost:
                        de_geom = QgsGeometry.fromPolyline([dn1_geom.asPoint(), dn2_geom.asPoint()])

                        # Calculating the primal edge costs
                        de_metric_cost = (0.5 * dn1_metric_cost) + (0.5 * dn2_metric_cost)
                        de_custom_cost = (0.5 * dn1_custom_cost) + (0.5 * dn2_custom_cost)

                        G.add_edge(dn1, dn2,
                                   geom=de_geom,
                                   angle_cost=180 - de_angle_cost,
                                   metric_cost=de_metric_cost,
                                   custom_cost=de_custom_cost)

    return G


def costNode(graph, cost, source, target):

    if cost == 'angle':
        # Determining neighboring node closest to target
        current_distance = nx.shortest_path_length(graph, source=source, target=target, weight='angle_cost')
        neighbors = graph[source]
        distances = []
        next_node = None
        for node in neighbors:
            distance = nx.shortest_path_length(graph, source=node, target=target, weight='angle_cost')
            # if distance < current_distance: # Don't choose 'rear' nodes
            distances.append(distance)
            if distance == min(distances): # Finding closest node
                next_node = node

        # Determining difference
        if len(distances) > 1:
            difference = max(distances) - min(distances)
        else:
            difference = 0


    elif cost == 'topological':
        # Determining neighboring node closest to target
        current_distance = nx.shortest_path_length(graph, source=source, target=target)
        neighbors = graph[source]
        distances = []
        next_node = None
        for node in neighbors:
            distance = nx.shortest_path_length(graph, source=node, target=target)
            if distance < current_distance:  # Don't choose 'rear' nodes
                distances.append(distance)
                if distance == min(distances):  # Finding closest node
                    next_node = node

        # Determining difference
        if len(distances) > 1:
            difference = max(distances) - min(distances)
        else:
            difference = 0

    elif cost == 'metric':
        # Determining neighboring node closest to target
        current_distance = nx.shortest_path_length(graph, source=source, target=target, weight='metric_cost')
        neighbors = graph[source]
        distances = []
        next_node = None
        for node in neighbors:
            distance = nx.shortest_path_length(graph, source=node, target=target, weight='metric_cost')
            distances.append(distance)
            if distance == min(distances):  # Finding closest node
                next_node = node

        # Determining difference
        if len(distances) > 1:
            difference = max(distances) - min(distances)
        else:
            difference = 0

    elif cost == 'custom':
        # Determining neighboring node closest to target
        current_distance = nx.shortest_path_length(graph, source=source, target=target, weight='custom_cost')

        neighbors = graph[source]
        distances = []
        next_node = None
        for node in neighbors:
            distance = nx.shortest_path_length(graph, source=node, target=target, weight='custom_cost')
            distances.append(distance)
            if distance == min(distances):  # Finding closest node
                next_node = node

        # Determining difference
        if len(distances) > 1:
            difference = max(distances) - min(distances)
        else:
            difference = 0

    return next_node, difference

def routeGraph(graph, source, target, ruling):
    path = [source, ]  # Travelled edges
    node = source
    length = 0
    while node != target:
        new_node = None
        for index, cost in enumerate(ruling):
            if not new_node:
                # Determine next node based on current cost
                potential_next_node, difference = costNode(graph, cost[0], node, target)

                # High proximity benefit and choice of route determine next node
                if difference > cost[1] or difference == 0:
                    if not potential_next_node in path:
                        length += graph[node][potential_next_node]['angle_cost']
                        node = potential_next_node
                        path.append(node)

                # Last rule determines next node
                elif index == (len(ruling) - 1):
                    length += graph[node][potential_next_node]['angle_cost']
                    node = potential_next_node
                    path.append(node)

    return path, length


def writeGraph(graph):
    # Create network layer
    vl = QgsVectorLayer("Linestring", "graph", "memory")
    pr = vl.dataProvider()

    # Create fields
    fields = []
    field_names = []
    for (u, v, d) in graph.edges_iter(data=True):
        for field_name in d.keys():
            if not field_name == 'geom':
                field_type = QVariant.Double
                field_names.append(field_name)
                fields.append(QgsField(field_name, field_type))
        break  # Only need one entry for getting the fields
    vl.startEditing()
    pr.addAttributes(fields)
    vl.updateFields()

    # Add features
    for node1, node2, data in graph.edges(data=True):
        # Adding attributes
        fet = QgsFeature(vl.pendingFields())
        for field_name in field_names:
            fet.setAttribute(field_name, data[field_name])
        # Adding geometry
        geom = data['geom']
        fet.setGeometry(geom)
        pr.addFeatures([fet])

    # Save and add to the canvas
    vl.commitChanges()
    vl.updateExtents()
    QgsMapLayerRegistry.instance().addMapLayer(vl)


def writeNodes(graph):
    # Create network layer
    vl = QgsVectorLayer("Point", "nodes", "memory")
    pr = vl.dataProvider()

    # Add id field
    vl.startEditing()
    pr.addAttributes([QgsField('id', QVariant.String), ])
    vl.updateFields()

    # Add features
    for index, (node, data) in enumerate(graph.nodes(data=True)):
        fet = QgsFeature(vl.pendingFields())
        fet.setAttribute('id', index)
        geom = data['geom']
        fet.setGeometry(geom)
        pr.addFeatures([fet])

    # Save and add to the canvas
    vl.commitChanges()
    vl.updateExtents()
    QgsMapLayerRegistry.instance().addMapLayer(vl)

def writePath(graph, path):
    # Create network layer
    vl = QgsVectorLayer("Linestring", "path", "memory")
    pr = vl.dataProvider()

    # Getting geometry from graph
    for node in path:
        fet = QgsFeature()
        geom = graph.node[node]['p_geom']
        fet.setGeometry(geom)
        pr.addFeatures([fet])

    # Save and add to the canvas
    vl.commitChanges()
    vl.updateExtents()
    QgsMapLayerRegistry.instance().addMapLayer(vl)


G = constructPrimalGraph(vectorlayer, 'CONN')
DG = constructDualGraph(G)
writeGraph(DG)
writeNodes(DG)

# Routing following the ruling
ruling = [('angle', 40)]
path, length = routeGraph(DG, DG.nodes()[886], DG.nodes()[2900], ruling)
writePath(DG, path)

# Routing using Dijkstra and angular cost
path2 = nx.shortest_path(DG, source=DG.nodes()[886], target=DG.nodes()[2900], weight='angle_cost')
print nx.shortest_path_length(DG, source=DG.nodes()[886], target=DG.nodes()[2900], weight='metric_cost  ')
writePath(DG, path2)

# Routing using Dijkstra and metric cost
path3 = nx.shortest_path(DG, source=DG.nodes()[886], target=DG.nodes()[2900], weight='metric_cost')
print nx.shortest_path_length(DG, source=DG.nodes()[886], target=DG.nodes()[2900], weight='metric_cost  ')
writePath(DG, path2)