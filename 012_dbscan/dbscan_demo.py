# DBSCAN demo, written in Python3  
# by Prof. Dr. Juergen Brauer, www.juergenbrauer.org
#
# Dependency: only one! QT for Python (module name: pyside6) for visualiziations
# Use
#
#   pip install pyside6
#
# if you haven't installed it yet.
#
# Note: in this implementation I assume that there is only
#       one unique data point for each location (x,y)
# Reason: I use a dictionary with the key (x,y) as
#         a data structure to store the cluster id and
#         data point type for each data point (x,y)


from platform import node
import PySide6.QtCore

# Prints PySide6 version
#print(PySide6.__version__)
# Prints the Qt version used to compile PySide6
#print(PySide6.QtCore.__version__)


import sys
import numpy
from PySide6 import QtCore, QtWidgets, QtGui, Qt
import random
import params
from dbscan_algorithm import dbscan



class MyVisualization(QtWidgets.QWidget):
    
    def __init__(self):
        super().__init__()        
        self.data_points = []
        self.setMouseTracking(True)

        self.dbscan_algorithm = None
        self.data_points_labels = None

        # generate 1000 cluster colors
        self.cluster_colors = [QtGui.QColor(0,0,0),
                               QtGui.QColor(255,0,0),
                               QtGui.QColor(0,255,0),
                               QtGui.QColor(0,0,255),
                               QtGui.QColor(0,255,255),
                               QtGui.QColor(128,128,128)
                               ]
        for i in range(1000-len(self.cluster_colors)):
            r = numpy.random.randint(low=0, high=255)
            g = numpy.random.randint(low=0, high=255)
            b = numpy.random.randint(low=0, high=255)
            self.cluster_colors.append( QtGui.QColor(r,g,b) )

        self.show_epsilon_neighborhoods = False
        self.nearest_point_to_mouse = None


    def update_and_show_mouse_pos(self, event):

        # window dimensions are not yet finished
        # in __init__ializer()
        # so we retrieve the current window dimensions here
        self.height = self.size().height()
        self.width  = self.size().width()

        # get mouse position
        pos = event.position().toPoint()
        self.mousex = pos.x()
        self.mousey = pos.y()
        
        # compute nearest point to mouse
        # only for this point we will display the epsilon-environment
        if len(self.data_points)>0:
                
            self.nearest_point_to_mouse = None
            mindist = None
            for p in self.data_points:
                dist = numpy.sqrt( (self.mousex-p[0])**2 + (self.mousey-p[1])**2 )
                if mindist == None or dist < mindist:
                    mindist = dist
                    self.nearest_point_to_mouse = p

        # show infos
        N = len(self.data_points)
        data_point_type = ""
        data_point_cluster_id = ""
        if self.data_points_labels != None:
            data_point_type       = self.data_points_labels[self.nearest_point_to_mouse]["data_point_type"]
            data_point_cluster_id = self.data_points_labels[self.nearest_point_to_mouse]["cluster_id"]
        self.setWindowTitle( f"mouse: ({self.mousex},{self.mousey}), data points: {N}\t\t"
                             f"type: {data_point_type}, cluster ID: {data_point_cluster_id}" )



    def generate_point(self, x,y):

        # if we set one new point,
        # f forget all DBSCAN clustering labels
        # produced previously
        self.data_points_labels = None

        # store the new point
        self.data_points.append( (x,y) )


    def react(self, event):
        self.update_and_show_mouse_pos(event)        
        if event.button() == QtCore.Qt.LeftButton:      
            self.generate_point(self.mousex, self.mousey)            
        elif event.button() == QtCore.Qt.RightButton:
            radius = 30
            for point_nr in range(params.RIGHT_MOUSE_CLICK_NR_POINTS_TO_GENERATE):
                rndx = self.mousex - radius + random.randint(0, 2*radius)
                rndy = self.mousey - radius + random.randint(0, 2*radius)
                rndx = numpy.clip(rndx, 0, self.width-1)
                rndy = numpy.clip(rndy, 0, self.height-1)
                self.generate_point(rndx, rndy)

        # induce re-drawing of the widget
        self.update()


    def mouseMoveEvent(self, event):                
        #print("move")
        self.react(event)        

    def mousePressEvent(self, event):
        #print("press")
        self.react(event)


    def keyPressEvent(self, event):

        keycode = event.key()
        try:
            c = chr(keycode)
        except:
            pass

        # run a single DBSCAN step
        if c=="R":

            self.dbscan_algorithm = dbscan(params.DBSCAN_EPSILON,
                                           params.DBSCAN_MINPTS)
            self.data_points_labels = self.dbscan_algorithm.cluster( self.data_points )            

            #print("\nClustering info:")
            #print(self.data_points_labels)


        # toggle on/off display of epsilon environments
        if c=="E":
            self.show_epsilon_neighborhoods = not self.show_epsilon_neighborhoods


        # clear all data points?
        if c=="C":
            self.data_points = []
            self.data_points_labels = None
            self.nearest_point_to_mouse = None
            self.show_epsilon_neighborhoods = None
            self.dbscan_algorithm = None
        


        # induce re-drawing of the widget
        self.update()

    
    def paintEvent(self, e):
        qp = QtGui.QPainter()
        qp.begin(self)        
        self.draw_all(qp)
        qp.end()

      
    
    def draw_all(self, qp):

        #print("draw_all")
        
        # prepare font
        font = QtGui.QFont()
        font.setFamily("Ubuntu")
        font.setPixelSize(17)
        font.setBold(False)
        qp.setFont(font)
      
        # data points are visualized by circles
        # what's the radius for this circle?
        r = params.VISU_DATA_POINT_RADIUS
        
        # draw all points
        for p in self.data_points:           
            
            # default color for data points is black
            col = QtGui.QColor(0,0,0)

            # did we already cluster the data points,
            # i.e., do we already have data point labels?
            if self.data_points_labels != None:

                # get cluster id for this data point
                cluster_id = self.data_points_labels[p]["cluster_id"]

                # get data point type
                data_point_type = self.data_points_labels[p]["data_point_type"]

                # map cluster_id to a color
                col = self.cluster_colors[ cluster_id ]
            

            # prepare pen and brush with
            # the right cluster-associated color
            pen = QtGui.QPen( col )
            qp.setPen(pen)
            brush = QtGui.QBrush( col )
            qp.setBrush(brush)

            # draw data point as filled circle
            qp.drawEllipse( QtCore.QPoint(*p), r, r)


        # show epsilon neighborhoods?
        if self.show_epsilon_neighborhoods and self.nearest_point_to_mouse != None:

            # prepare black pen
            pen = QtGui.QPen( QtGui.QColor(0,0,0) )
            qp.setPen(pen)
            
            # we don't want to the circles
            # to be drawn as filled circles,
            # so unset the brush
            qp.setBrush(QtCore.Qt.NoBrush)
            
            # draw circle around that point
            qp.drawEllipse( QtCore.QPoint(*self.nearest_point_to_mouse),
                            params.DBSCAN_EPSILON,
                            params.DBSCAN_EPSILON)
            
            



manual = \
f"""
DBSCAN demo by Prof. Dr. Juergen Brauer, www.juergenbrauer.org

Click
    left mouse button to generate one point
    right mouse button to generate {params.RIGHT_MOUSE_CLICK_NR_POINTS_TO_GENERATE} points

Press  
  c clear data points
  r to run DBSCAN
  e to display the epsilon-environment of data point next to mouse  
"""

print(manual)

app = QtWidgets.QApplication([])
widget = MyVisualization()
widget.resize(800, 800)
widget.show()
sys.exit(app.exec())