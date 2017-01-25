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

    print nx.number_of_edges(G)


    return G

def constructDualGraph(primalGraph):
    # Create undirected empty graph
    G = nx.Graph()

    # Create dual graph nodes
    for pn1, pn2, data in primalGraph.edges_iter(data=True):
        
        # Dual graph node
        dn = (pn1,pn2)
        
        # Calculating azimuth of dual graph node
        pn1_geom = QgsPoint(data['geom'][0])
        pn2_geom = QgsPoint(data['geom'][1])
        dn_azimuth = pn1_geom.azimuth(pn2_geom)

        # Node geometry based on centroid
        dn_geom = QgsGeometry.fromPolyline([QgsPoint(pn1), QgsPoint(pn2)]).centroid().asPoint()

        G.add_node(dn, geom=dn_geom, azimuth=dn_azimuth, metric_cost=data['geom'], custom_cost=data['custom_cost'])

        # Determining the neighboring dual graph nodes

        pe1_neighbors = [(pn1, neighbor) for neighbor in primalGraph[pn1] if neighbor != pn2]
        pe2_neighbors = [(pn2, neighbor) for neighbor in primalGraph[pn2] if neighbor != pn1]
        dn_neighbors = list(set(pe1_neighbors + pe2_neighbors))

        # Looping through neighbors and creating the edges
        for dn_neighbor in dn_neighbors:

            # Creating dual graph edge geometry
            pn1_neighbor = QgsPoint(dn_neighbor[0])
            pn2_neighbor = QgsPoint(dn_neighbor[1])
            dn_neighbor_geom = QgsGeometry.fromPolyline([pn1_neighbor, pn2_neighbor]).centroid().asPoint()
            de_geom = QgsGeometry.fromPolyline([dn_geom, dn_neighbor_geom])

            # Calculating dual graph edge angular cost based on primal edges
            dn_azimuth_neighbor = dn_neighbor[0].azimuth(dn_neighbor[1])
            angle = 360 - abs(dn_azimuth - dn_azimuth_neighbor)
            if angle > 180:  # wide angles
                de_angle_cost = 180 - (360 - abs(angle))
            else:  # acute angles
                de_angle_cost = 180 - angle

            if G.has_node(dn) and G.has_node(dn_neighbor):
                G.add_edge(dn, dn_neighbor, geom=de_geom, angle_cost=de_angle_cost)

    print nx.number_of_nodes(G)
    print G.nodes()

    return G

def writeGraph(graph):
    # Create network layer
    vl = QgsVectorLayer("Linestring", "graph", "memory")
    pr = vl.dataProvider()

    # Create fields
    fields = []
    field_names = []
    for (u,v,d) in graph.edges_iter(data=True):
        for field_name in d.keys():
            if not field_name == 'geom':
                field_type =  QVariant.Double
                field_names.append(field_name)
                fields.append(QgsField(field_name, field_type))
        break # Only need one entry for getting the fields
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




G = constructPrimalGraph(vectorlayer, 'id')
DG = constructDualGraph(G)
writeGraph(DG)
