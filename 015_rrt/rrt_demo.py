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
        
        self.show_window_title()


    def show_window_title(self):

        txt = f"mouse: ({self.mousex},{self.mousey})"
        if self.rrt_algorithm != None:
            txt += f" - RRT step #{self.rrt_algorithm.step}," \
                   f" nodes: {len(self.rrt_algorithm.tree.nodes)}"
            self.setWindowTitle( txt )
  

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

        self.rrt_algorithm = rrt(map, start, goal)



    def keyPressEvent(self, event):

        keycode = event.key()
        try:
            c = chr(keycode)
        except:
            pass

        # run a single RRT step
        if c=="R":

            if self.rrt_algorithm == None:
                self.try_to_init_rrt_algorithm(self.map,
                                            self.location_start,
                                            self.location_goal)

            if self.rrt_algorithm != None:
                self.rrt_algorithm.run_single_step()


        # run several RRT steps
        if c=="T":

            if self.rrt_algorithm == None:
                self.try_to_init_rrt_algorithm(self.map,
                                            self.location_start,
                                            self.location_goal)

            if self.rrt_algorithm != None:
                for i in range(params.DEMO_RUN_N_STEPS):
                    self.rrt_algorithm.run_single_step()



        # clear, i.e. re-initialize, algorithm?
        if c=="C":
            print("clear")

            self.try_to_init_rrt_algorithm(self.map,
                                           self.location_start,
                                           self.location_goal)
        
        # induce re-drawing of the widget
        self.show_window_title()
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

        # 1. 
        # first draw map of the world
        # and then everything else on top
        # of this image
        qp.drawImage(0,0, self.map)


        # 2. draw start location - if already defined
        if self.location_start != None:
            col = QtGui.QColor(*params.VISU_COLOR_LOCATION_START)
            pen = QtGui.QPen( col )
            qp.setPen(pen)
            brush = QtGui.QBrush( col )
            qp.setBrush(brush)
            r = params.VISU_LOCATION_START_RADIUS
            qp.drawEllipse( QtCore.QPoint(*self.location_start), r, r)

        # 3. draw goal location - if already defined
        if self.location_goal != None:
            col = QtGui.QColor(*params.VISU_COLOR_LOCATION_GOAL)
            pen = QtGui.QPen( col )
            qp.setPen(pen)
            brush = QtGui.QBrush( col )
            qp.setBrush(brush)
            r = params.VISU_LOCATION_GOAL_RADIUS
            qp.drawEllipse( QtCore.QPoint(*self.location_goal), r, r)

            # draw termination circle around goal location
            col = QtGui.QColor(*params.VISU_COLOR_TERMINATION_CIRCLE)
            pen = QtGui.QPen( col )
            qp.setPen(pen)
            qp.setBrush(QtCore.Qt.NoBrush)            
            r = params.ALGO_TERMINATION_RADIUS
            qp.drawEllipse( QtCore.QPoint(*self.location_goal), r, r)


        # 4. draw the tree
        if self.rrt_algorithm != None:

            
            r = params.VISU_NODE_RADIUS

            for node in self.rrt_algorithm.tree.nodes:

                # draw this node
                col = QtGui.QColor(*params.VISU_COLOR_TREE_NODE)
                pen = QtGui.QPen( col )
                qp.setPen(pen)
                brush = QtGui.QBrush( col )
                qp.setBrush(brush)
                qp.drawEllipse( QtCore.QPoint(node.x, node.y), r, r)              

                # draw the edge to the parent node
                if node.parent != None:

                    col = QtGui.QColor(*params.VISU_COLOR_TREE_EDGE)
                    pen = QtGui.QPen( col )
                    qp.setPen(pen)
                    brush = QtGui.QBrush( col )
                    qp.setBrush(brush)

                    qp.drawLine(node.x, node.y,
                                node.parent.x, node.parent.y)


            if self.rrt_algorithm.last_rnd_point != None:

                p = self.rrt_algorithm.last_rnd_point

                # draw last random point used in the RRT algorithm
                # to augment the tree
                col = QtGui.QColor(*params.VISU_COLOR_LAST_RANDOM_POINT)
                pen = QtGui.QPen( col )
                qp.setPen(pen)
                brush = QtGui.QBrush( col )
                qp.setBrush(brush)

                r = params.VISU_LAST_RANDOM_POINT_RADIUS            
                qp.drawEllipse( QtCore.QPoint(p[0],p[1]), r, r)


            # did we already find a path?
            if self.rrt_algorithm.path_from_start_to_goal != None:

                last_loc = None

                # yes, we did!
                # draw that path!
                for loc in self.rrt_algorithm.path_from_start_to_goal:

                    # draw path node
                    col = QtGui.QColor(*params.VISU_COLOR_FOUND_PATH_NODE)
                    pen = QtGui.QPen( col )
                    qp.setPen(pen)
                    brush = QtGui.QBrush( col )
                    qp.setBrush(brush)
                    r = params.VISU_NODE_RADIUS
                    qp.drawEllipse( QtCore.QPoint(loc[0],loc[1]), r, r)

                    if last_loc != None:

                        # draw path edge
                        col = QtGui.QColor(*params.VISU_COLOR_FOUND_PATH_EDGE)
                        pen = QtGui.QPen( col,
                                          params.VISU_FOUND_PATH_WIDTH,
                                          QtCore.Qt.DashLine )
                        qp.setPen(pen)
                        brush = QtGui.QBrush( col )
                        qp.setBrush(brush)
                        qp.drawLine(last_loc[0], last_loc[1],
                                    loc[0], loc[1])

                    last_loc = loc




            
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
  t - to run {params.DEMO_RUN_N_STEPS} RRT steps
  c - to clear the RRT tree, i.e. to restart RRT algorithm
"""

print(manual)

app = QtWidgets.QApplication([])
widget = MyVisualization()
widget.resize(800, 800)
widget.show()
sys.exit(app.exec())