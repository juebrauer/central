# RRT - Rapidly-exploring Random Tress demo, written in Python3  
# by Prof. Dr. Juergen Brauer, www.juergenbrauer.org
#
# Dependency: only one! QT for Python (module name: pyside6) for visualiziations
# Use
#
#   pip install pyside6
#
# if you haven't installed it yet.


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
from rrt_algorithm import rrt



class MyVisualization(QtWidgets.QWidget):
    
    def __init__(self):
        super().__init__()        
        self.data_points = []
        self.setMouseTracking(True)

        self.location_start = None
        self.location_goal = None
        self.rrt_algorithm = None

        # load in the map (occupancy grid)
        # as an image
        self.map = QtGui.QImage("world1.png")
       


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
        
        # show infos
        self.setWindowTitle( f"mouse: ({self.mousex},{self.mousey})" )


  

    def react(self, event):
        self.update_and_show_mouse_pos(event)
        if event.button() == QtCore.Qt.LeftButton:      
            self.location_start = (self.mousex, self.mousey)
        elif event.button() == QtCore.Qt.RightButton:
            self.location_goal = (self.mousex, self.mousey)

        # induce re-drawing of the widget
        self.update()


    def mouseMoveEvent(self, event):                
        self.react(event)        

    def mousePressEvent(self, event):
        self.react(event)


    def try_to_init_rrt_algorithm(self, map, start, goal):

        if self.location_start == None:
            print("Before running RRT, you have to define the start location!")
            return

        if self.location_goal == None:
            print("Before running RRT, you have to define the goal location!")
            return

        if self.rrt_algorithm == None:
            self.rrt_algorithm = rrt(map, start, goal)



    def keyPressEvent(self, event):

        keycode = event.key()
        try:
            c = chr(keycode)
        except:
            pass

        # run a single DBSCAN step
        if c=="R":

            self.try_to_init_rrt_algorithm(self.map, self.location_start, self.location_goal)

            if self.rrt_algorithm != None:
                self.graph = self.rrt_algorithm.run_single_step()


        # clear, i.e. re-initialize, algorithm?
        if c=="C":

            self.try_to_init_rrt_algorithm(self.map, self.location_start, self.location_goal)
        
        # induce re-drawing of the widget
        self.update()

    
    def paintEvent(self, e):
        qp = QtGui.QPainter()
        qp.begin(self)        
        self.draw_all(qp)
        qp.end()

      
    
    def draw_all(self, qp):

        # prepare font
        font = QtGui.QFont()
        font.setFamily("Ubuntu")
        font.setPixelSize(17)
        font.setBold(False)
        qp.setFont(font)

        # first draw map of the world
        # and then everything else on top
        # of this image
        qp.drawImage(0,0, self.map)


        if self.location_start != None:
            col = QtGui.QColor(255,0,0)
            pen = QtGui.QPen( col )
            qp.setPen(pen)
            brush = QtGui.QBrush( col )
            qp.setBrush(brush)
            r = params.VISU_LOCATION_START_RADIUS
            qp.drawEllipse( QtCore.QPoint(*self.location_start), r, r)

        if self.location_goal != None:
            col = QtGui.QColor(0,0,255)
            pen = QtGui.QPen( col )
            qp.setPen(pen)
            brush = QtGui.QBrush( col )
            qp.setBrush(brush)
            r = params.VISU_LOCATION_GOAL_RADIUS
            qp.drawEllipse( QtCore.QPoint(*self.location_goal), r, r)
        
    # end-def draw_all


            



manual = \
f"""
RRT - Rapidly-exploring random trees demo
by Prof. Dr. Juergen Brauer, www.juergenbrauer.org

Click
  left mouse  - to set start location
  right mouse - to set goal location

Press
  r - to run a single RRT step
  c - to clear, i.e. re-initialize, RRT algorithm
"""

print(manual)

app = QtWidgets.QApplication([])
widget = MyVisualization()
widget.resize(800, 800)
widget.show()
sys.exit(app.exec())