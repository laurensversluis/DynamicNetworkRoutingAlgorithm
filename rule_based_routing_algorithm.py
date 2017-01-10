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


class primalGraph():

    def __init__(self, vectorLayer, costField):
        self.vectorLayer = vectorLayer
        self.costField = costField


    def getNetworkPoints(self):
        processedPoints = set()
        networkPoints = []
        for feature in self.vectorLayer.getFeatures():
            points = feature.geometry().asPolyline()
            if not all(p in processedPoints for p in points in):
                processedPoints.add(point for point in points)
                networkPoints.extend(points)

        return networkPoints


    def getUndirectedGraph(self, networkPoints):
        graph = None
        tiedPoints = []
        if self.vectorLayer:
            networkFields = vectorLayer.pendingFields()
            networkCostIndex = networkFields.indexFromName(costField)
            director = QgsLineVectorLayerDirector(vectorLayer, -1, '', '', '', 3)
            properter = customCostProperter(networkCostIndex, 0)
            director.addProperter(properter)
            builder = QgsGraphBuilder(vectorLayer.crs())
            tiedPoints = director.makeGraph(builder, self.getNetworkPoints()) # All nodes are tied into the graph
            graph = builder.graph()

        return graph, tiedPoints


class dualGraph():

    def __init__(self, primalGraph, tiedPoints):
        self.primalGraph = primalGraph
        self.tiedPoints = tiedPoints

        # iterate over features
    def getConnectedEdges(self, vertexId):

        inEdgeIds = self.primalGraph.vertex(vertexId).inArc()
        outEdgeIds = self.primalGraph.vertex(vertexId).outArc()
        conEdgeIds = [inEdgeIds, outEdgeIds]

        return conEdgeIds


    def getConnectedVertices(self, vertexId):

        connEdgeIds = self.getConnectedEdges(vertexId)

        connVertexIds = set()

        for edgeId in connEdgeIds:
            inVertexId = self.primalGraph.arc(edgeId).inVertexId
            outVertexId = self.primalGraph.arc(edgeId).outVertexId
            if (inVertexId or outVertexId) != vertexId:
                if not (inVertexId or outVertexId) in connVertexIds:
                    connVertexIds.add(inVertexId)
                    connVertexIds.add(outVertexId)

        return connVertexIds

    def getAngularCost(self, vertexId):

        vertexAngularCosts = []

        np = self.primalGraph.vertex(vertexId).point()

        connVertexIds = self.getConnectedVertices(vertexId)

        for vertexStart in connVertexIds:
            vp1 = self.primalGraph.vertex(vertex).point()
            vp1Angle = vp1.azimuth(np)
            for vertexEnd in connVertexIds:
                if vertexEnd != vertexStart:
                    vp2 = self.primalGraph.vertex(vertex).point()
                    vp2Angle = vp2.azimuth(np)
                    # Calculating the angle between two edges
                    angle = 360 - abs(vp1Angle - vp2Angle)
                    if angle > 180: # wide angles
                        angleCost = 180 - (360 - abs(angle))
                    else: # acute angles
                        angleCost = 180 - angle

            vertexAngularCosts[(vertexStart,vertexEnd)] = angleCost

        return vertexAngularCosts


class analysis():

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


    def __init__(self, graph, tiedPoints, radius):
        self.graph = graph
        self.radius = radius
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