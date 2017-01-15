# Dependencies
import networkx as nx



# Add edges and nodes based on dual graph type
def constructPrimalGraph(vectorlayer, custom_cost_field):
    # Create empty graph
    G = nx.Graph()

    # Read segments and build dualgraph
    for segment in vectorlayer.getFeatures():
        metric_cost = segment.length()
        custom_cost = segment[custom_cost_field]
        # Determine primal nodes
        seg_start = segment.asPolyline()[0]
        seg_end = segment.asPolyline()[1]
        G.add_edge(seg_start, seg_end, 'azimuth'=azimuth, 'metric_cost'=metric_cost, 'custom_cost'=custom_cost)

    return G

def constructDualGraph(primalGraph):
    # Create empty graph
    G = nx.Graph()
    # Read edges and add as nodes to graph
    for (start_node,end_node,attributes) in primalGraph.edges():
        start_neighbors = primalGraph.neighbors(start_node)
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
