# DBSCAN demo, written in Python3  
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
from PySide6 import QtCore, QtWidgets, QtGui
import random
import params



class MyVisualization(QtWidgets.QWidget):
    
    def __init__(self):
        super().__init__()        
        self.points = []
        self.setMouseTracking(True)        


    def update_and_show_mouse_pos(self, event):

        # window dimensions are not yet finished
        # in __init__ializer()
        # so we retrieve the current window dimensions here
        self.height = self.size().height()
        self.width  = self.size().width()

        pos = event.position().toPoint()
        self.mousex = pos.x()
        self.mousey = pos.y()
        self.setWindowTitle( f"{self.mousex},{self.mousex}" )


    def generate_point(self, x,y):
        self.points.append( (x,y) )


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
            pass

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

        pen = QtGui.QPen( QtGui.QColor(0,0,0) )
        qp.setPen(pen)

        brush = QtGui.QBrush( QtGui.QColor(0,0,0) )
        qp.setBrush(brush)

        qp.setFont(font)
      
        
        # draw all points
        for p in self.points:
            
            r = params.VISU_DATA_POINT_RADIUS
            qp.drawEllipse( QtCore.QPoint(*p), r, r)


manual = \
"""
DBSCAN demo by Prof. Dr. Juergen Brauer, www.juergenbrauer.org

Click
    left mouse button to generate one point
    right mouse button to generate many points

Press  
  r to run DBSCAN
"""

print(manual)

app = QtWidgets.QApplication([])
widget = MyVisualization()
widget.resize(800, 800)
widget.show()
sys.exit(app.exec())