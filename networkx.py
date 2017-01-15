# Dependencies
import networkx as nx



# Add edges and nodes based on dual graph type
def constructDualGraph(vectorlayer):
    # Create empty graph
    g = nx.Graph()

    # Read segments and build dualgraph
    for segment in vectorlayer.getFeatures():


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
