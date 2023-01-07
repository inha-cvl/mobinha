"""
pip install PyOpenGL PyOpenGL_accelerate
https://github.com/pyside/Examples/blob/master/examples/opengl/samplebuffers.py
"""
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
import sys
import math

import selfdrive.visualize.obj.objloader as loader

from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *


class ImuGL(QOpenGLWidget):
	def __init__(self, parent=None):
		super(ImuGL, self).__init__(parent)
		self.roll = 0.0
		self.pitch = 0.0
		self.yaw = 0.0

	def initializeGL(self):
		glutInit()
		glLightfv(GL_LIGHT0, GL_POSITION, (-40, 200, 100, 0.0))
		glLightfv(GL_LIGHT0, GL_AMBIENT, (0.5, 0.5, 0.5, 1.0))
		glLightfv(GL_LIGHT0, GL_DIFFUSE, (1, 1, 1, 1.0))
		glEnable(GL_LIGHT0)
		glEnable(GL_LIGHTING)
		glEnable(GL_DEPTH_TEST)
		glEnable(GL_COLOR_MATERIAL)
		glShadeModel(GL_SMOOTH)
		glClearColor(0, 0, 0, 1.0)
		glMatrixMode(GL_PROJECTION)
		glLoadIdentity()
		gluPerspective(45, 320/float(210), 1, 100.0)
		glEnable(GL_DEPTH_TEST)
		glMatrixMode(GL_MODELVIEW)

		#self.car = loader.ObjLoader()

	def paintGL(self):
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
		glLoadIdentity()
		glPushMatrix()
		glScalef(0.3, 0.4, 0.4)
		glTranslatef(0.3, -0.3, -5.5)
		glRotatef(30, 1, 0, 0)
		glRotatef(-45, 0, 1, 0)

		self.drawLine()

		glRotatef(self.roll, 1, 0, 0)
		glRotatef(self.pitch, 0, 1, 0)
		#glCallList(self.car.gl_list)
		glPopMatrix()
		#self.DrawRect(1.0, 1.0, 1.0)

	@pyqtSlot(float, float, float)
	def updateRP(self, roll_change, pitch_change, yaw_change):
		self.roll = roll_change
		self.pitch = pitch_change
		self.yaw = yaw_change
		self.update()

	def drawLine(self):
		xroll = "R_{}".format(round(self.roll, 3))
		ypitch = "P_{}".format(round(self.pitch, 3))
		zyaw = "Y_{}".format(round(self.yaw, 3))

		glPushMatrix()

		glPushMatrix()
		glColor3f(1, 0, 0)
		glBegin(GL_LINES)
		glVertex3f(5.0, 0.0, 0.0)
		glVertex3f(-5.0, 0.0, 0.0)
		glEnd()
		self.drawBitmapText(xroll, -5.0, 0.0, 0.0)
		glPopMatrix()

		glPushMatrix()
		glColor3f(0, 1, 0)
		glBegin(GL_LINES)
		glVertex3f(0.0, 5.0, 0.0)
		glVertex3f(0.0, -5.0, 0.0)
		glEnd()
		self.drawBitmapText(ypitch, 0.0, 2.0, -0.2)
		glPopMatrix()

		glPushMatrix()
		glColor3f(0, 0, 1)
		glBegin(GL_LINES)
		glVertex3f(0.0, 0.0, 5.0)
		glVertex3f(0.0, 0.0, -5.0)
		glEnd()
		self.drawBitmapText(zyaw, 0.0, 0.5, 3)
		glPopMatrix()

		glPopMatrix()

		glFlush()

	def drawBitmapText(self, str, x, y, z):
		glRasterPos3f(x, y, z)
		for ch in str:
			glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, ctypes.c_int(ord(ch)))

	def resizeGL(self, width, height):
		glGetError()
		aspect = width if (height == 0) else width / height
		glViewport(0, 0, width, height)
		glMatrixMode(GL_PROJECTION)
		glLoadIdentity()
		gluPerspective(45, aspect, 1, 100.0)
		glMatrixMode(GL_MODELVIEW)
		glLoadIdentity()

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
