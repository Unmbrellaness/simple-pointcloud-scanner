import sys
import open3d as o3d
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout
from PyQt5.QtOpenGL import QGLWidget



class PointCloudWidget(QGLWidget):
    def __init__(self, parent=None):
        super(PointCloudWidget, self).__init__(parent)
        self.point_cloud = None

    def set_point_cloud(self, point_cloud):
        self.point_cloud = point_cloud
        self.update()

    def paintGL(self):
        if self.point_cloud:
            o3d.visualization.draw_geometries([self.point_cloud])


class MainWindow(QMainWindow):
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.setWindowTitle("Point")

        self.point_cloud_widget = PointCloudWidget(self)
        self.setCentralWidget(self.point_cloud_widget)

    def set_point_cloud(self, point_cloud):
        self.point_cloud_widget.set_point_cloud(point_cloud)


if __name__ == '__main__':
    # 读取点云数据
    pcd_path = 'data/p1_2000.ply'
    point_cloud = o3d.io.read_point_cloud(pcd_path)

    # 创建PyQt5应用程序
    app = QApplication(sys.argv)
    window = MainWindow()

    # 设置点云数据到窗口中
    window.set_point_cloud(point_cloud)

    # 显示窗口
    window.show()

    # 运行应用程序
    sys.exit(app.exec_())
