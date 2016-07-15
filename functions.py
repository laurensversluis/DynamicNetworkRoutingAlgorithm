

def makeUndirectedGraph(network_layer, points=list):
    graph = None
    tied_points = []
    if network_layer:
        director = QgsLineVectorLayerDirector(network_layer, -1, '', '', '', 3)
        properter = QgsDistanceArcProperter()
        director.addProperter(properter)
        builder = QgsGraphBuilder(network_layer.crs())
        tied_points = director.makeGraph(builder, points)
        graph = builder.graph()
    return graph, tied_points


def makeDirectedGraph(network_layer, points=list, direction_field=-1, one_way='', reverse_way='', two_way='', default_direction=3):
    graph = None
    tied_points = []
    if network_layer:
        director = QgsLineVectorLayerDirector(network_layer, direction_field, one_way, reverse_way, two_way, default_direction)
        properter = QgsDistanceArcProperter()
        director.addProperter(properter)
        builder = QgsGraphBuilder(network_layer.crs())
        tied_points = director.makeGraph(builder, points)
        graph = builder.graph()
    return graph, tied_points


def calculateRouteTree(graph, tied_points, origin, destination, impedance=0):
    points = []
    if tied_points:
        try:
            from_point = tied_points[origin]
            to_point = tied_points[destination]
        except:
            return points

        # analyse graph
        if graph:
            form_id = graph.findVertex(from_point)
            tree = QgsGraphAnalyzer.shortestTree(graph, form_id, impedance)
            form_id = tree.findVertex(from_point)
            to_id = tree.findVertex(to_point)

            # iterate to get all points in route
            if to_id == -1:
                pass
            else:
                while form_id != to_id:
                    l = tree.vertex(to_id).inArc()
                    if not l:
                        break
                    e = tree.arc(l[0])
                    points.insert(0, tree.vertex(e.inVertex()).point())
                    to_id = e.outVertex()

                points.insert(0, from_point)

    return points


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


def calculateServiceArea(graph, tied_points, origin, cutoff, impedance=0):
    points = {}
    if tied_points:
        try:
            from_point = tied_points[origin]
        except:
            return points

        # analyse graph
        if graph:
            from_id = graph.findVertex(from_point)

            (tree, cost) = QgsGraphAnalyzer.dijkstra(graph, from_id, impedance)

            i = 0
            while i < len(cost):
                if cost[i] <= cutoff and tree[i] != -1:
                    points[str(i)]=((graph.vertex(i).point()),cost)
                i += 1

    return points

