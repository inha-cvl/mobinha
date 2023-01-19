"""
pip install PyOpenGL PyOpenGL_accelerate
https://github.com/pyside/Examples/blob/master/examples/opengl/samplebuffers.py
"""
import os
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
import pywavefront
import random
import math
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
import time

dir_path = str(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))


class ObjLoader():

    def __init__(self):
        self.scene = pywavefront.Wavefront(
            dir_path+'/obj/Car.obj', create_materials=True, collect_faces=True)
        #self.mtl = MTL()

    def model(self):
        glPushMatrix()

        for mesh in self.scene.mesh_list:
            glBegin(GL_TRIANGLES)
            for face in mesh.faces:
                g = self.random_color()
                glColor3f(g, g, g)
                for vertex_i in face:
                    glVertex3f(*self.scene.vertices[vertex_i])
            glEnd()
        glPopMatrix()

    def random_color(self):
        r = (random.randrange(120, 200))/255
        # g = (random.randrange(1, 256))/255
        # b = (random.randrange(1, 256))/255
        return r


class ImuGL(QOpenGLWidget):
    def __init__(self, parent=None):
        super(ImuGL, self).__init__(parent)
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0 
    def initializeGL(self):
        glutInit()
        self.obj_loader = ObjLoader()

        glClearColor(0, 0, 0, 0.0)
        glShadeModel(GL_SMOOTH)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_COLOR_MATERIAL)
        glLoadIdentity()
        gluPerspective(120, 1, 6, 100)

        glLightfv(GL_LIGHT0, GL_POSITION, (100, 100, 100, 1.0))
        glLightfv(GL_LIGHT0, GL_AMBIENT, (0.8, 0.8, 0.8, 1.0))
        glLightfv(GL_LIGHT0, GL_DIFFUSE, (1.0, 1.0, 1.0, 1.0))
        glLightfv(GL_LIGHT0, GL_SPECULAR, (1.0, 1.0, 1.0, 1.0))

    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

        glPushMatrix()
        glTranslatef(0, -0.9, -6.0)
        # glRotatef(-30, 0, 1, 0)
        # glRotatef(3, 1, 0, 0)

        glRotatef(self.roll, 0, 0, 1)
        glRotatef(self.pitch+3, 1, 0, 0)
        glRotatef(self.yaw+30, 0, 1, 0)

        self.drawLine()
        self.obj_loader.model()

        glPopMatrix()

    def updateRP(self, roll_change, pitch_change, yaw_change):
        self.roll = roll_change
        self.pitch = pitch_change
        self.yaw = yaw_change
        self.update()

    def drawLine(self):
        xroll = "R {}".format(round(self.roll, 3))
        ypitch = "P {}".format(round(self.pitch, 3))
        zyaw = "Y {}".format(round(self.yaw, 3))

        glPushMatrix()

        glPushMatrix()
        glColor3f(0, 1, 0)
        glBegin(GL_LINES)
        glVertex3f(3.0, 0.7, 0.0)
        glVertex3f(-3.5, 0.7, 0.0)
        glEnd()
        self.drawBitmapText(ypitch, 2.8, 0.5, 0.0)
        glPopMatrix()

        glPushMatrix()
        glColor3f(1, 0, 0)
        glBegin(GL_LINES)
        glVertex3f(0.0, 3.0, 0.3)
        glVertex3f(0.0, -1.0, 0.3)
        glEnd()
        self.drawBitmapText(zyaw, 0.2, 3.0, 0)
        glPopMatrix()

        glPushMatrix()
        glColor3f(0, 0, 1)
        glBegin(GL_LINES)
        glVertex3f(0.0, 0.7, 3.5)
        glVertex3f(0.0, 0.7, -15.0)
        glEnd()
        self.drawBitmapText(xroll, 0.0, 0.6, 3.5)
        glPopMatrix()

        glPopMatrix()

        glFlush()

    def drawBitmapText(self, str, x, y, z):
        glRasterPos3f(x, y, z)
        for ch in str:
            glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18,
                                ctypes.c_int(ord(ch)))

    def resizeGL(self, width, height):
        glGetError()
        aspect = width if (height == 0) else width / height
        glViewport(0, 0, width, height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, aspect, 1, 100.0)
        glMatrixMode(GL_MODELVIEW)

    def DrawRect(self, x, y, z):
        point1 = [x/2.0, y/2.0, z/-2.0]
        point2 = [x/2.0, y/2.0, z/2.0]
        point3 = [x/2.0, y/-2.0, z/2.0]
        point4 = [x/2.0, y/-2.0, z/-2.0]
        point5 = [x/-2.0, y/-2.0, z/2.0]
        point6 = [x/-2.0, y/2.0, z/2.0]
        point7 = [x/-2.0, y/2.0, z/-2.0]
        point8 = [x/-2.0, y/-2.0, z/-2.0]

        glBegin(GL_QUADS)

        glVertex3fv(point1)
        glVertex3fv(point2)
        glVertex3fv(point6)
        glVertex3fv(point7)

        glVertex3fv(point3)
        glVertex3fv(point4)
        glVertex3fv(point8)
        glVertex3fv(point5)

        glVertex3fv(point2)
        glVertex3fv(point3)
        glVertex3fv(point5)
        glVertex3fv(point6)

        glVertex3fv(point7)
        glVertex3fv(point8)
        glVertex3fv(point4)
        glVertex3fv(point1)

        glVertex3fv(point6)
        glVertex3fv(point5)
        glVertex3fv(point8)
        glVertex3fv(point7)

        glVertex3fv(point1)
        glVertex3fv(point4)
        glVertex3fv(point3)
        glVertex3fv(point2)

        glEnd()
