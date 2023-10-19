import sys
import program.learningPyUI as learningPyUI
import program.BlueTeeth as blueTeeth
from PyQt5.QtCore import QThread, pyqtSignal
import serial
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import threading
from qt_material import apply_stylesheet
import registration as r2

from PyQt5.QtWidgets import QApplication, QMainWindow, QFileDialog, QDialog

class Receive(QThread):
    finished = pyqtSignal()

    def __init__(self):
        super(Receive, self).__init__()
        self._stop_event = threading.Event()
        self.ser = None
        self.data_list = []
        self.save_path = r"result0.pcd"
        self.lock = threading.Lock()        # 扫描点云保存路径锁，用于主程序修改
        # self.list_lock = threading.Lock()       # 数据锁，用于获取当前扫描内容
        self.counter = 0

    def set_port(self,ser):
        self.ser = ser

    def set_savepath(self,savepath):
        global save_path
        self.lock.acquire()
        try:
            self.save_path = savepath
        finally:
            self.lock.release()

    # def get_data_list(self,data_list_temp):
    #     global data_list
    #     self.list_lock.acquire()
    #     try:
    #         data_list_temp = self.data_list
    #     finally:
    #         self.list_lock.acquire()


    def read_serial_and_save_list(self):

        if self.ser.in_waiting:  # 如果串口有数据可读
            line = self.ser.readline().decode().strip()  # 读取一行数据并转换为字符串
            if line == 'end':
                global save_path
                self.lock.acquire()
                try:
                    print(line)
                    pcd = o3d.geometry.PointCloud()
                    pcd.points = o3d.utility.Vector3dVector(self.data_list)
                    o3d.io.write_point_cloud(self.save_path, pcd)
                    o3d.visualization.draw_geometries([pcd])
                finally:
                    self.lock.release()

            elif line == 'waiting...':
                print(line)
            elif line == 'out':
                print(line)
                self.data_list.clear()
            elif line == 'begin':
                print(line)
            else:
                nums = list(map(float, line.split()))  # 将字符串按空格分割成列表
                self.data_list.append(nums)  # 将列表中的元素添加到数组中
                # print(nums)  # 打印读取到的数据


    def stop(self):
        self._stop_event.set()

    def run(self):
        while True:
            self.read_serial_and_save_list()


class MainWindow(learningPyUI.Ui_MainWindow, QMainWindow):
    def __init__(self):
        learningPyUI.Ui_MainWindow.__init__(self)
        QMainWindow.__init__(self)
        self.setupUi(self)
        self.ser = None
        self.receive_t = None
        self.childWindow = None
        # self.receive_t.set_port(self.ser)

    def setChildWindow(self, childWindow):
        self.childWindow = childWindow


    def create_thread(self):
        if(self.ser != None):
            self.receive_t = Receive()
            # self.receive_t.set_port(self.ser)
            self.receive_t.start()

    def connect_port(self):
        self.childWindow.label.setText("waiting...")
        self.childWindow.show()
        try:
            if(self.ser == None):
                self.ser = serial.Serial('COM9', 115200, timeout=0)
                self.create_thread()
                self.receive_t.set_port(self.ser)
                # save_path = "DemoICPPointClouds/result3.pcd"
                if self.ser.isOpen():
                    self.childWindow.label.setText("succeed")
                    print("open success")
        except:
            self.childWindow.label.setText("failed")
            print("error")

    def sendStart(self):
        # 判断是否有串口连接
        if(self.ser == None):
            print("connect error1")
            return 0
        # print("jinlaile")
        # 生成信号
        X_begin = self.Xbegin.text()
        Y_begin = self.Ybegin.text()
        X_step = self.Xstep.text()
        Y_step = self.Ystep.text()
        X_end = self.Xend.text()
        Y_end = self.Yend.text()
        X_judge = self.Xjudge.text()
        Y_judge = self.Yjudge.text()
        Save_path = self.filePathlineEdit1.text()
        print(Save_path)

        self.receive_t.set_savepath(Save_path)
        beginSignal = "1 "+ X_begin + " " + Y_begin + " " + X_step + " " + Y_step + " " + X_end + " " + Y_end + " " + X_judge + " " + Y_judge
        print("已生成信号：", beginSignal)
        self.ser.write(beginSignal.encode())

    def stop(self):
        if(self.ser == None):
            print("connect error2")
            return 0
        self.ser.write(b"2")


    def regStart(self):
        print("regStart....")
        src_path = self.filePathlineEdit2.text()
        dst_path = self.filePathlineEdit3.text()
        voxel_size = float(self.Voxel_size.text())
        distance_multiplier = float(self.Distance_multiplier.text())
        max_iterations = int(self.Iterations.text())
        confidence = float(self.Confidence.text())
        threshold = float(self.Threshold.text())
        tar_path = self.filePathlineEdit5.text()
        # 开始粗配准
        result_c_trans = r2.coarse_registration(src_path,dst_path,voxel_size,distance_multiplier,max_iterations,confidence)

        # 精配准
        r2.fine_registration(src_path, dst_path, tar_path, result_c_trans.transformation, threshold)

    def clusStart(self):

        print("Cluster Start...")
        src_path = str(self.filePathlineEdit4.text())
        eps = float(self.lineEdit_2.text())
        min_points = int(self.lineEdit_3.text())

        # print(src_path)
        # print(float(eps))
        # print(int(min_points))
        pcd = o3d.io.read_point_cloud(src_path)
        # Flip it, otherwise the pointcloud will be upside down.
        pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])

        with o3d.utility.VerbosityContextManager(
                o3d.utility.VerbosityLevel.Debug) as cm:
            labels = np.array(
                pcd.cluster_dbscan(eps, min_points, print_progress = True))

        max_label = labels.max()
        print(f"point cloud has {max_label + 1} clusters")
        colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
        colors[labels < 0] = 0
        pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
        o3d.visualization.draw([pcd])


        # 开始聚类
        # cl.clustering(src_path, eps, min_points)

    def openFile_source1(self):
        print("openFile_source1")
        get_directory_path = QFileDialog.getExistingDirectory(self,
                                    "选取指定文件夹",
                                    "C:/")
        self.filePathlineEdit1.setText(str(get_directory_path))

    def openFile_source2(self):
        print("openFile_source2")
        get_filename_path, ok = QFileDialog.getOpenFileName(self,
                                    "选取单个文件",
                                   "C:/",
                                    "All Files (*);;Text Files (*.txt)")
        if ok:
            self.filePathlineEdit2.setText(str(get_filename_path))

    def openFile_source3(self):
        print("openFile_source3")
        get_filename_path, ok = QFileDialog.getOpenFileName(self,
                                    "选取单个文件",
                                   "C:/",
                                    "All Files (*);;Text Files (*.txt)")
        if ok:
            self.filePathlineEdit3.setText(str(get_filename_path))

    def openFile_source4(self):
        print("openFile_source4")
        get_filename_path, ok = QFileDialog.getOpenFileName(self,
                                    "选取单个文件",
                                   "C:/",
                                    "All Files (*);;Text Files (*.txt)")
        if ok:
            self.filePathlineEdit4.setText(str(get_filename_path))

    def openFile_source5(self):
        print("openFile_source5")
        get_directory_path = QFileDialog.getExistingDirectory(self,
                                    "选取指定文件夹",
                                    "C:/")
        self.filePathlineEdit5.setText(str(get_directory_path))


class ChildWindow(QDialog, blueTeeth.Ui_BlueTeethConnection):
    def __init__(self):
        blueTeeth.Ui_BlueTeethConnection.__init__(self)
        QDialog.__init__(self)
        self.setupUi(self)

        self.setWindowTitle('Bluez Message')
        # self.pushButton.clicked.connect(self.btnClick)  # 按钮事件绑定

    def reject(self):  # 子窗体自定义事件
        print("reject")
        self.hide()

    def accept(self):  # 子窗体自定义事件
        print("accept")
        self.hide()


if __name__ == '__main__':

    #
    # ser = serial.Serial('COM9', 115200, timeout=0)
    # save_path = "DemoICPPointClouds/result3.pcd"
    #
    # if ser.isOpen():
    #     print("open success")
    # else:
    #     print("open failed")


    print("1")
    app = QApplication(sys.argv)  # 创建应用程序对象
    dow = MainWindow()  # 创建主窗口
    blue_connection = ChildWindow()

    print("2")
    # setup stylesheet
    # apply_stylesheet(app, theme='light_amber.xml',)

    print("3")
    dow.setChildWindow(blue_connection)


    print("5")
    dow.show()  # 显示主窗口

    sys.exit(app.exec_())  # 在主线程中退出
