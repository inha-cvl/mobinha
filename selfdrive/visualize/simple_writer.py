from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5 import uic
import os
import time

dir_path = str(os.path.dirname(os.path.realpath(__file__)))
form_class = uic.loadUiType(dir_path+"/forms/simple_writer.ui")[0]


class SimpleWriter(QMainWindow, form_class):
    def __init__(self, path='', parent=None):
        super(SimpleWriter, self).__init__(parent)
        self.setupUi(self)
        self.path = path
        self.save_button.clicked.connect(self.save_file)

    def save_file(self):
        contents = self.textEdit.toPlainText()
        with open(self.path, "w") as f:
            f.write(contents)
        time.sleep(0.5)
        self.close()
