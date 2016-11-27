# Importing modules
from PyQt4.QtCore import *
from PyQt4.QtGui import *

from qgis.core import *
from qgis.gui import *
from qgis.networkanalysis import *

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


class network():

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
            tiedPoints = director.makeGraph(builder, networkPoints)
            graph = builder.graph()
        return graph, tiedPoints


class analysis():

    def __init__(self, graph, tiedPoints, radius):
        self.graph = graph
        self.radius = radius
        self.results = {graph.findVertex(tied_point): 0 for tied_point in tiedPoints}

    def calculateTreeDijkstra(self, toId, impedance=0):

        # Point to id

        (tree, cost) = QgsGraphAnalyzer.dijkstra(self.graph, toId, impedance)
        (tree, cost) = QgsGraphAnalyzer.dijkstra(self.graph, toId, impedance)
        #Find common points


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