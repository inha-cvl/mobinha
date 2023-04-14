import os
from PyQt5.QtCore import QUrl
from PyQt5.QtWidgets import QApplication
from PyQt5.QtMultimedia import QMediaPlayer, QMediaContent

dir_path = str(os.path.dirname(os.path.realpath(__file__)))

app = QApplication([])
player = QMediaPlayer()
url = QUrl.fromLocalFile(dir_path+"/sounds/off.wav")
media = QMediaContent(url)
player.setMedia(media)
player.setVolume(50)
player.play()
app.exec_()