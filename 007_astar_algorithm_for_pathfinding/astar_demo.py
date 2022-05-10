# A* demo, written in Python3  
# by Prof. Dr. Juergen Brauer, www.juergenbrauer.org
#
# Visualization is handled here directly in class MyGrid
# A* algorithm implementation is in class astar in file astar_algorithm.py
#
# Dependency: only one! QT for Python (module name: pyside6) for visualizing a grid
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
from PySide6 import QtCore, QtWidgets, QtGui
import cell_types
import astar_node_types
from astar_algorithm import astar



class MyGrid(QtWidgets.QWidget):
    
    def __init__(self, grid_height, grid_width ):
        super().__init__()

        self.grid_height = grid_height
        self.grid_width = grid_width        
        self.grid = numpy.zeros( (self.grid_height, self.grid_width) )

        self.start = None
        self.goal  = None
        self.astar_algorithm = None

        self.setMouseTracking(True)



        #self.label = QtWidgets.QLabel("Test")        
        #self.layout = QtWidgets.QVBoxLayout(self)
        #self.layout.addWidget(self.label)

    def update_current_selected_cell(self, event):

        pos = event.position().toPoint()
        my = pos.y()
        mx = pos.x()        
        self.setWindowTitle( f"{mx},{my}" )

        # what is the corresponding grid cell coordinate?        
        self.cell_y = int( my // self.gridcellvisu_height )
        self.cell_x = int( mx // self.gridcellvisu_width )


    def react(self, event):

        self.update_current_selected_cell(event)
        
        if event.button() == QtCore.Qt.LeftButton:            
            #print( f"cell coordinates: {self.cell_x}, {self.cell_y}" )
            self.grid[ self.cell_y, self.cell_x ] = cell_types.celltype_wall
            
        elif event.button() == QtCore.Qt.RightButton:            
            #print( f"cell coordinates: {self.cell_x}, {self.cell_y}" )
            self.grid[ self.cell_y, self.cell_x ] = cell_types.celltype_empty

        # induce re-drawing of the widget
        self.update()


    def mouseMoveEvent(self, event):                
        #print("move")
        self.react(event)        

    def mousePressEvent(self, event):
        #print("press")
        self.react(event)


    def initialize_or_reinitialize_astar_algorithm(self):

        # we need to know the start end node
        # in order to start with the A* algorithm!
        if self.start == None or self.goal == None:
            return

        # create A* algorithm instance
        self.astar_algorithm = astar(map=self.grid,
                                     start=self.start,
                                     goal=self.goal)


    def keyPressEvent(self, event):

        keycode = event.key()
        try:
            c = chr(keycode)
        except:
            pass

        # set start position
        if c=="S":
            self.start = (self.cell_x, self.cell_y)
            self.initialize_or_reinitialize_astar_algorithm()

        # set goal position
        if c=="G":
            self.goal = (self.cell_x, self.cell_y)
            self.initialize_or_reinitialize_astar_algorithm()

        # run a single A* algorithm step
        if c=="R":

            if self.start == None:
                print("You first have to set the start node before I can run A*")
                
            if self.goal == None:
                print("You first have to set the goal node before I can run A*")
                
            if self.start != None and self.goal != None:
                # run a single A* iteration
                self.node_infos_from_astar_algorithm = self.astar_algorithm.single_step()

        # clear old path search and restart?
        if c=="C":
            print("Clearing old path search. Restarting A* algorithm.")
            self.initialize_or_reinitialize_astar_algorithm()
        
        # induce re-drawing of the widget
        self.update()

    
    def paintEvent(self, e):
        qp = QtGui.QPainter()
        qp.begin(self)        
        self.draw_grid(qp)
        qp.end()


    def draw_cell(self, qp, visux, visuy, c):

        # set pen and brush color
        qp.setPen( QtGui.QColor(0,0,0) )        
        qp.setBrush( c  )
        
        qp.drawRect(visux,
                    visuy,
                    self.gridcellvisu_width,
                    self.gridcellvisu_height)

       
    
    def draw_grid(self, qp):

        #print("drawGrid")

        # how large is the widget?
        h = self.size().height()
        w = self.size().width()
                
        # how large can we visualize one grid cell?
        self.gridcellvisu_height = h // self.grid_height
        self.gridcellvisu_width  = w // self.grid_width

        # prepare font
        font = QtGui.QFont()
        font.setFamily("Ubuntu")
        font.setPixelSize(17)
        font.setBold(False)
        pen = QtGui.QPen( QtGui.QColor(0,0,0) )
        qp.setPen(pen)
        qp.setFont(font)      
      
        
        # draw all grid cells
        for cell_y in range(0, self.grid_height):
            for cell_x in range(0, self.grid_width):

                # compute left top corner (x,y) of
                # drawing rectangle
                visu_x = cell_x * self.gridcellvisu_width
                visu_y = cell_y * self.gridcellvisu_height

                                
                grid_cell_type = self.grid[ cell_y, cell_x ]                    

                # determine grid cell color
                rgb_values = (255,255,255)

                # is there already an A* algorithm instance?
                if self.astar_algorithm != None:
                    node_infos = self.astar_algorithm.node_infos[(cell_x,cell_y)]
                    astar_node_type = node_infos["node_type"]                    
                    if astar_node_type == astar_node_types.nodetype_open:
                        rgb_values = (0,255,0) # green
                    if astar_node_type == astar_node_types.nodetype_closed:
                        rgb_values = (200,200,200) # light gray
                    if astar_node_type == astar_node_types.nodetype_path:
                        rgb_values = (255,255,0) # yellow

                if (cell_x, cell_y) == self.start:
                    rgb_values = (255,0,0) # red
                if (cell_x, cell_y) == self.goal:
                    rgb_values = (100,100,255) # light blue
                if grid_cell_type == cell_types.celltype_wall:
                    rgb_values = (128,128,128) # dark gray
                
                c = QtGui.QColor( *rgb_values )
                self.draw_cell(qp, visu_x, visu_y, c)

                # if there is enough space,
                # display grid cell coordinates
                # and f,g,h costs                
                if self.gridcellvisu_width>=60 and \
                   self.gridcellvisu_height>=60:
                
                    qp.drawText(visu_x+self.gridcellvisu_width//3 - 20,
                                visu_y+self.gridcellvisu_height//3,
                                f"({cell_x},{cell_y})")

                    
                    if self.astar_algorithm != None:
                        node_infos = self.astar_algorithm.node_infos[(cell_x,cell_y)]
                        costs_f = node_infos["f"]
                        costs_g = node_infos["g"]
                        costs_h = node_infos["h"]                    
                        qp.drawText(visu_x+self.gridcellvisu_width//3 - 20,
                                    visu_y+self.gridcellvisu_height//3 + 20,
                                    f"{costs_f}={costs_g}+{costs_h}")


manual = \
"""
A* demo by Prof. Dr. Juergen Brauer, www.juergenbrauer.org

Press
  s: to set start node
  g: to set goal node
  r: to run a single A* iteration
  c: to restart the A* algorithm

In the code, change grid_height and grid_width in order
to make the grid larger or smaller.

If the display size for a grid cell is large -
which is the case for small  grid sizes -
the demo also displays the cell coordinates and
the f=g+h costs in each cell.
"""

print(manual)

app = QtWidgets.QApplication([])
widget = MyGrid(grid_height=10, grid_width=10)
widget.resize(1000, 1000)
widget.show()
sys.exit(app.exec())