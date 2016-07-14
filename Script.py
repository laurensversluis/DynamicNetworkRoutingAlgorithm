from qgis.core import *
from qgis.networkanalysis import *
from qgis.utils import *


# Load Network
network = QgsVectorLayer("R:/RND_Projects/Project/RND073_QGIS_Toolkit/RND073_Project_Work/RND073_Axial/RND073_Existing/ae_network.shp",
                               "network",
                               "ogr")

