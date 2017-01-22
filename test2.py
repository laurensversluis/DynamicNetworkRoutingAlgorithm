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
        pn1 = geom.asPolyline()[0]
        pn2 = geom.asPolyline()[1]
        metric_cost = geom.length()
        custom_cost = segment[custom_cost_field]
        # Determine primal nodes
        G.add_edge(pn1, pn2, geom=geom.asPolyline(), metric_cost=metric_cost, custom_cost=custom_cost)

    return G


def normaliseAngle(azimuth):
    if azimuth < 180:
        angle = azimuth
    else:
        angle = 180 - azimuth

    return angle


def constructDualGraph(primalGraph):
    # Create undirected empty graph
    G = nx.Graph()
    de_list = []

    # Create dual graph nodes
    for pn1, pn2, data in primalGraph.edges_iter(data=True):

        # Calculating azimuth of dual graph node
        pn1_geom = QgsPoint(data['geom'][0])
        pn2_geom = QgsPoint(data['geom'][1])
        dn_azimuth = pn1_geom.azimuth(pn2_geom)

        # Node geometry based on centroid
        dn_geom = QgsGeometry.fromPolyline([QgsPoint(pn1), QgsPoint(pn2)])

        # Creating graph node
        dn = dn_geom.centroid().asPoint()
        G.add_node(dn,
                   geom=dn_geom,
                   azimuth=dn_azimuth,
                   metric_cost=data['metric_cost'],
                   custom_cost=data['custom_cost'])

    # Creating dual graph edges
    for pn1, data in primalGraph.nodes_iter(data=True):

        # Find neighboring dual graph nodes
        neighbors = [pn2 for pn2 in primalGraph[pn1]]

        # Getting the first neighbor and getting its dual node
        for pn2 in neighbors:
            dn1 = QgsGeometry.fromPolyline([QgsPoint(pn1), QgsPoint(pn2)]).centroid().asPoint()
            dn1_geom = G.node[dn1]['geom']
            dn1_azimuth = G.node[dn1]['azimuth']
            dn1_metric_cost = G.node[dn1]['metric_cost']
            dn1_custom_cost =G.node[dn1]['custom_cost']
            dn1_angle = normaliseAngle(dn1_azimuth)

            # Getting the second neighbor and getting its dual node
            for pn3 in neighbors:
                dn2 = QgsGeometry.fromPolyline([QgsPoint(pn1), QgsPoint(pn3)]).centroid().asPoint()
                dn2_geom = G.node[dn2]['geom']
                dn2_azimuth = G.node[dn2]['azimuth']
                dn2_metric_cost = G.node[dn2]['metric_cost']
                dn2_custom_cost = G.node[dn2]['custom_cost']
                dn2_angle = normaliseAngle(dn2_azimuth)

                # Calculating the angular cost between the dual nodes / primal edges
                angle = 180 - abs(dn1_azimuth - dn2_azimuth)
                if angle < 0:
                    de_angle_cost = abs(angle)
                else:
                    de_angle_cost = 180 - angle

                # Generating the dual edge geometry
                de_geom = QgsGeometry.fromPolyline([dn1_geom.asPoint(), dn2_geom.asPoint()])

                # Calculating the primal edge costs
                de_metric_cost = (0.5 * dn1_metric_cost) + (0.5 * dn2_metric_cost)
                de_custom_cost = (0.5 * dn1_custom_cost) + (0.5 * dn2_custom_cost)

                G.add_edge(dn1, dn2,
                           geom=de_geom,
                           angle_cost=de_angle_cost,
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
        for index, node in enumerate(neighbors):
            distance = nx.shortest_path_length(graph, source=node, target=target, weight='angle_cost')
            if distance < current_distance: # Don't choose 'rear' nodes
                distances.append(distance)
                if distance == min(distances): # Finding closest node
                    next_node = node


        # Determining difference
        if len(distances) > 1:
            difference_ratio = (max(distances) - min(distances)) / max(distances)
        else:
            difference_ratio = 0


    elif cost == 'topological':
        # Determining neighboring node closest to target
        current_distance = nx.shortest_path_length(graph, source=source, target=target)
        neighbors = graph[source]
        distances = []
        next_node = None
        for index, node in enumerate(neighbors):
            distance = nx.shortest_path_length(graph, source=node, target=target)
            if distance < current_distance:  # Don't choose 'rear' nodes
                distances.append(distance)
                if distance == min(distances):  # Finding closest node
                    next_node = node


        # Determining difference
        if len(distances) > 1:
            difference_ratio = (max(distances) - min(distances)) / max(distances)
        else:
            difference_ratio = 0

    elif cost == 'metric':
        # Determining neighboring node closest to target
        current_distance = nx.shortest_path_length(graph, source=source, target=target, weight='metric_cost')
        neighbors = graph[source]
        distances = []
        next_node = None
        for node in neighbors:
            distance = nx.shortest_path_length(graph, source=node, target=target, weight='metric_cost')
            if distance < current_distance:  # Don't choose 'rear' nodes
                distances.append(distance)
                if distance == min(distances):  # Finding closest node
                    next_node = node

        # Determining difference
        if len(distances) > 1:
            difference_ratio = (max(distances) - min(distances)) / max(distances)
        else:
            difference_ratio = 0

    elif cost == 'custom':
        # Determining neighboring node closest to target
        current_distance = nx.shortest_path_length(graph, source=source, target=target, weight='metric_cost')
        neighbors = graph[source]
        distances = []
        next_node = None
        for index, node in enumerate(neighbors):
            distance = nx.shortest_path_length(graph, source=node, target=target, weight='metric_cost')
            if distance < current_distance:  # Don't choose 'rear' nodes
                distances.append(distance)
                if distance == min(distances):  # Finding closest node
                    next_node = node

        # Determining difference
        if len(distances) > 1:
            difference_ratio = (max(distances) - min(distances)) / max(distances)
        else:
            difference_ratio = 0

    return next_node, difference_ratio

def routeGraph(graph, source, target, ruling):
    path = [source, ]  # Travelled edges

    node = source
    while node != target:
        next_node = None
        for cost in ruling['priority']:
            if not next_node:
                potential_next_node, difference_ratio = costNode(graph, cost, node, target)
            if difference_ratio < ruling['switch_ratio']:
                next_node = potential_next_node
        path.append(next_node)
        node = next_node

    return path


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


def writePath(graph, path):
    # Create network layer
    vl = QgsVectorLayer("Linestring", "path", "memory")
    pr = vl.dataProvider()

    # Getting geometry from graph
    for node in path:
        fet = QgsFeature()
        geom = graph.node[node]['geom']
        fet.setGeometry(geom)
        pr.addFeatures([fet])

    # Save and add to the canvas
    vl.commitChanges()
    vl.updateExtents()
    QgsMapLayerRegistry.instance().addMapLayer(vl)



G = constructPrimalGraph(vectorlayer, 'NACHrN')
DG = constructDualGraph(G)
writeGraph(DG)
ruling = {'priority': ['angle', 'metric', 'custom'], 'switch_ratio': 0.1}
path = routeGraph(DG,DG.nodes()[0],DG.nodes()[-1],ruling)
writePath(DG, path)