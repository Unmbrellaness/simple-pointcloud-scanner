from PyQt5.QtCore import QThread, pyqtSignal
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton
import time
import serial

class Worker(QThread):
    finished = pyqtSignal()

    def __init__(self):
        self.ser = serial.Serial('COM9', 115200, timeout=0)
        if self.ser.isOpen():
            print("open success")
        else:
            print("open failed")

    def read_serial_and_save_list(self):
        if self.ser.in_waiting:  # 如果串口有数据可读
            line = self.ser.readline().decode().strip()  # 读取一行数据并转换为字符串
            print(line)  # 打印读取到的数据


    def run(self):
        while True:
            self.read_serial_and_save_list(self.ser)


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.button = QPushButton('Start', self)
        self.button.clicked.connect(self.start_worker)
        self.setCentralWidget(self.button)

        self.worker = Worker()
        self.worker.finished.connect(self.worker_finished)


    def start_worker(self):
        self.button.setEnabled(False)
        self.worker.start()

    def worker_finished(self):
        self.button.setEnabled(True)
        print('Worker finished')

if __name__ == '__main__':
    app = QApplication([])
    window = MainWindow()




    window.show()
    app.exec_()