# Importing modules
from PyQt4.QtCore import *
from PyQt4.QtGui import *

from qgis.core import *
from qgis.gui import *
from qgis.networkanalysis import *

import math

# Creating replacement for the standard network analysis proporter
class customCostProperter(QgsArcProperter):
    def __init__(self, costColumIndex, defaultValue):
        QgsArcProperter.__init__(self)
        self.cost_column_index = costColumIndex
        self.default_value = defaultValue

    def property(self, distance, feature):
        cost = float(feature.attributes()[self.cost_column_index])
        if cost <= 0.0:
            return QVariant(self.default_value)
        return cost

    def requiredAttributes(self):
        l = []
        l.append(self.cost_column_index);
        return l


class Graph:

    def __init__(self, vectorLayer, cost='length'):
        self.vectorLayer = vectorLayer
        self.costField = cost


    def getNetworkPoints(self):
        processedPoints = set()
        networkPoints = []
        for feature in self.vectorLayer.getFeatures():
            points = feature.geometry().asPolyline()
            if not all(p in processedPoints for p in points):
                processedPoints.add(point for point in points)
                networkPoints.extend(points)

        return networkPoints


    def getUndirectedGraph(self):
        graph = None
        tiedPoints = []
        if self.vectorLayer:
            networkFields = self.vectorLayer.pendingFields()
            networkCostIndex = networkFields.indexFromName(self.costField)
            director = QgsLineVectorLayerDirector(self.vectorLayer, -1, '', '', '', 3)
            properter = customCostProperter(networkCostIndex, 0)
            director.addProperter(properter)
            builder = QgsGraphBuilder(self.vectorLayer.crs())
            tiedPoints = director.makeGraph(builder, self.getNetworkPoints()) # All nodes are tied into the graph
            graph = builder.graph()

        return graph, tiedPoints


class DualGraph:

    def __init__(self, vectorLayer, cost='angular'):
        self.vectorLayer = vectorLayer
        self.costType = cost

    def indexNetwork(self):
        segmentIndex = QgsSpatialIndex()
        segmentDict = {}
        for index, feature in enumerate(self.vectorLayer.getFeatures()):
            segmentIndex.insertFeatures(feature)
            segmentDict[index] = feature.geometry()

        return segmentIndex, segmentDict

    def getNetworkVertices(self):
        processedPoints = set()
        networkPoints = []
        for feature in self.vectorLayer.getFeatures():
            points = feature.geometry().asPolyline()
            if not all(p in processedPoints for p in points):
                processedPoints.add(point for point in points)
                networkPoints.extend(points)

        return networkPoints

    # Create pairs where segments connect
    def getNetworkEdges(self):
        networkEdges = []
        segmentIndex, segmentDict = self.indexNetwork()
        for index, geom in segmentDict:
            connectedEdgeIds = segmentIndex.intersects(geom.boundingBox()) # get connected edges
            for id in connectedEdgeIds:
                if not (id, index) in networkEdges: # check if reverse edge is already there
                    networkEdges.append((index,id))

        return networkEdges


    def calculateAngularCost(self, id1, id2):

        segmentDict = self.indexNetwork()[1]

        segGeom1 = segmentDict[id1]
        segGeom2 = segmentDict[id2]

        # Calculating the abgle of the individual segments
        segAngle1 = segGeom1[0].azimuth(segGeom1[1])
        segAngle2 = segGeom2[0].azimuth(segGeom2[1])

        # Calculating the angle between two edges
        angle = 360 - abs(segAngle1 - segAngle2)
        if angle > 180:  # wide angles
            angleCost = 180 - (360 - abs(angle))
        else:  # acute angles
            angleCost = 180 - angle

        return angleCost


    def createGraph(self):
        graph = QgsGraph()
        vertices = self.getNetworkVertices()

        # Add vertices to the graph
        graph.addVertex(vertex for vertex in vertices)

        # Add edges to the graph
        for id1, id2 in self.getNetworkEdges():
            cost = self.calculateAngularCost(id1, id2)
            graph.addArc(id1, id2, cost)

        return graph




class analysis(primalGraph, dualGraph):

    # Base the analysis on the dual graph if there is one. Otherwise base it on a primal graph.

    # If it is a dual graph angular and topological costs are considered.

    # If it is a primal graph length and other custom costs are considered.

    # Metric and other costs are halved for each segment

    # For each type of costs different trees need to be made.

    # Each arc of each tree is linked by their geometry.

    # Analysis start by initialising the base graph (primal or dual)

    # It then continues by picking an origin and a destination

    # This origin forms the base for all the trees

    # Routing algorithms starts at the origin and considers all the trees costs to work out the best route.

    # Lookup function to get a vertexId using another tree's vertexId.

    #


    def __init__(self, vectorLayer, graphType, primalCosts, dualCosts):
        self.vectorLayer = vectorLayer
        self.graphType = graphType
        primalGraph.__init__(self, vectorLayer, costField=)
        self.results = {graph.findVertex(tied_point): 0 for tied_point in tiedPoints}

    def getTreeDijkstra(self, toId, impedance=0):
        (tree, cost) = QgsGraphAnalyzer.dijkstra(self.graph, toId, impedance)
        return tree, cost
        #Find common points

    def createCostTree(self, treeList):

        for tree, cost in treeList
            dic

    dict = {node: }

    def calculateBetweenness(self) :

        processedFromIds = []

        for fromId in range(len(cost)):
            if tree[fromId] == -1:
                pass
            elif fromId in processedFromIds:
                pass
            else:
                pathCount = 1
                curPos = fromId
                while curPos != to_id:
                    self.results[curPos] += pathCount
                    pathCount += 1
                    processedFromIds.append(curPos)
                    curPos = self.graph.arc(tree[curPos]).outVertex()


    def analysis(self):
        for vertex_id in range(10):
            calculateTreeDijkstra(self.graph, vertex_id, self.radius, self.results)
        return self.results