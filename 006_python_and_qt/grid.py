from calendar import c
import PySide6.QtCore

# Prints PySide6 version
print(PySide6.__version__)
# Prints the Qt version used to compile PySide6
print(PySide6.QtCore.__version__)


import sys
import random
import numpy
from PySide6 import QtCore, QtWidgets, QtGui


class MyGrid(QtWidgets.QWidget):

    celltype_empty = 0
    celltype_wall  = 1
    celltype_start = 2
    celltype_goal  = 3
    
    def __init__(self, grid_height, grid_width ):
        super().__init__()

        self.grid_height = grid_height
        self.grid_width = grid_width        
        self.grid = numpy.zeros( (self.grid_height, self.grid_width) )

        self.gridcell_colors = {}
        self.gridcell_colors[MyGrid.celltype_empty]  = (255,255,255)
        self.gridcell_colors[MyGrid.celltype_wall]   = (128,128,128)
        self.gridcell_colors[MyGrid.celltype_start]  = (255,0,0)
        self.gridcell_colors[MyGrid.celltype_goal]   = (0,0,255)

        self.start = None
        self.goal  = None


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
        self.cell_x = int( mx // self.gridcellvisu_width )
        self.cell_y = int( my // self.gridcellvisu_height )


    def react(self, event):

        self.update_current_selected_cell(event)
        
        if event.button() == QtCore.Qt.LeftButton:            
            print( f"cell coordinates: {self.cell_x}, {self.cell_y}" )
            self.grid[ self.cell_y, self.cell_x ] = MyGrid.celltype_wall
            
        elif event.button() == QtCore.Qt.RightButton:            
            print( f"cell coordinates: {self.cell_x}, {self.cell_y}" )
            self.grid[ self.cell_y, self.cell_x ] = MyGrid.celltype_empty

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

        if c=="S":
            self.start = (self.cell_x, self.cell_y)

        if c=="G":
            self.goal = (self.cell_x, self.cell_y)

        # induce re-drawing of the widget
        self.update()

    
    def paintEvent(self, e):
        qp = QtGui.QPainter()
        qp.begin(self)        
        self.draw_grid(qp)
        qp.end()


    def draw_cell(self, qp, visux, visuy, celltype):

        # set pen and brush color
        qp.setPen( QtGui.QColor(0,0,0) )
        rgb_values = self.gridcell_colors[ celltype ]
        c = QtGui.QColor( *rgb_values )                
        qp.setBrush( c  )
        
        qp.drawRect(visux,
                    visuy,
                    self.gridcellvisu_width,
                    self.gridcellvisu_height)

        #qp.drawText(visux+self.gridcellvisu_width//2,
        #            visuy+self.gridcellvisu_height//2)

    
    def draw_grid(self, qp):

        #print("drawGrid")

        # how large is the widget?
        h = self.size().height()
        w = self.size().width()
                
        # how large can we visualize one grid cell?
        self.gridcellvisu_height = h // self.grid_height
        self.gridcellvisu_width  = w // self.grid_width        

        # draw all grid cells
        for cell_y in range(0, self.grid_height):
            for cell_x in range(0, self.grid_width):

                # what is the current state of this cell?
                celltype = self.grid[cell_y, cell_x]

                # compute left top corner (x,y) of
                # drawing rectangle
                visu_x = cell_x * self.gridcellvisu_width
                visu_y = cell_y * self.gridcellvisu_height

                self.draw_cell(qp, visu_x, visu_y, celltype)


        # highlight start cell?
        if self.start != None:
            visu_x = self.start[0] * self.gridcellvisu_width
            visu_y = self.start[1] * self.gridcellvisu_height
            self.draw_cell(qp, visu_x, visu_y, MyGrid.celltype_start)

        # highlight goal cell?
        if self.goal != None:
            visu_x = self.goal[0] * self.gridcellvisu_width
            visu_y = self.goal[1] * self.gridcellvisu_height
            self.draw_cell(qp, visu_x, visu_y, MyGrid.celltype_goal)
     
            

app = QtWidgets.QApplication([])
widget = MyGrid(grid_height=20, grid_width=20)
widget.resize(800, 800)
widget.show()
sys.exit(app.exec())