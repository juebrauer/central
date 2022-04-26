import PySide6.QtCore

# Prints PySide6 version
print(PySide6.__version__)

# Prints the Qt version used to compile PySide6
print(PySide6.QtCore.__version__)


import sys
import random
from PySide6 import QtCore, QtWidgets, QtGui


class MyGrid(QtWidgets.QWidget):
    
    def __init__(self, grid_width, grid_height ):
        super().__init__()

        self.grid_width = grid_width
        self.grid_height = grid_height

        self.setMouseTracking(True)

        #self.label = QtWidgets.QLabel("Test")        
        #self.layout = QtWidgets.QVBoxLayout(self)
        #self.layout.addWidget(self.label)


    def mouseMoveEvent(self, event):
        p = event.localPos()
        pos = p.toPoint()
        mx = pos.x()
        my = pos.y()
        self.setWindowTitle( f"{mx},{my}" )

    def mousePressEvent(self, event):        
        if event.button() == QtCore.Qt.LeftButton:
            print( "LMB click at: ", event.position().x(), event.position().y() )
        elif event.button() == QtCore.Qt.RightButton:
            print( "RMB click at: ", event.position().x(), event.position().y() )

        self.update()
                
    
    def paintEvent(self, e):
        qp = QtGui.QPainter()
        qp.begin(self)
        #self.drawPoints(qp)
        self.drawGrid(qp)
        qp.end()


    def drawGrid(self, qp):

        print("drawGrid")

        # how large is the widget?
        w = self.size().width()
        h = self.size().height()
        
        # how large can we make one grid cell?
        gridcell_width  = w // self.grid_width
        gridcell_height = h // self.grid_height

        # draw all grid cells
        cell_nr = 0
        qp.setPen( QtGui.QColor(255,0,0) )
        for cell_y in range(0, self.grid_height):
            for cell_x in range(0, self.grid_width):
                x = cell_x * gridcell_width
                y = cell_y * gridcell_height

                qp.drawRect(x,y,gridcell_width, gridcell_height)

                qp.drawText(x+gridcell_width//3,
                            y+gridcell_height//2,
                            f"{cell_nr}")

                cell_nr += 1
        


    def drawPoints(self, qp):
        qp.setPen( QtGui.QColor(255,0,0) )

        size = self.size()
        for i in range(0,100):
            x = random.randint(1, size.width()-1)
            y = random.randint(1, size.height()-1)
            #qp.drawPoint(x,y)
            qp.drawEllipse(40,40,400,400)
            

app = QtWidgets.QApplication([])
widget = MyGrid(20,20)
widget.resize(800, 800)
widget.show()
sys.exit(app.exec())