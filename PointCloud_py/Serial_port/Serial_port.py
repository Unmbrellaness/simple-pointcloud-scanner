import serial
import threading
import registration_2 as regi
import open3d as o3d
import time

def read_serial_and_save_list(ser, data_list, path):
    while True:
        if ser.in_waiting:  # 如果串口有数据可读
            line = ser.readline().decode().strip()  # 读取一行数据并转换为字符串
            if line == 'end':
                print(line)
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(data_list)
                o3d.visualization.draw_geometries([pcd])
                o3d.io.write_point_cloud(path, pcd)
                continue
            elif line == 'waiting...':
                print(line)
                continue
            elif line == 'out':
                print(line)
                data_list.clear()
                continue
            elif line == 'begin':
                print(line)
                continue
            else:
                nums = list(map(float, line.split()))  # 将字符串按空格分割成列表
                data_list.append(nums)  # 将列表中的元素添加到数组中
                # print(nums)  # 打印读取到的数据

def format_floats(*args):
    return ' '.join('{:.1f}'.format(x) for x in args)

# def print_test(x_begin ,y_begin ,x_step ,y_step ,x_end ,y_end ,x_judge ,y_judge):
#     com_str = "1 "+ format_floats(x_begin ,y_begin ,x_step ,y_step ,x_end ,y_end ,x_judge ,y_judge)
#     print(com_str)



def send_command_begin(ser, x_begin ,y_begin ,x_step ,y_step ,x_end ,y_end ,x_judge ,y_judge):
    com_str = "1 "+ format_floats(x_begin ,y_begin ,x_step ,y_step ,x_end ,y_end ,x_judge ,y_judge)
    ser.write(com_str.encode())
    print("begin command send success")

def send_command_stop(ser):
    ser.write(b"2")
    print("begin command send success")

# 接收数据
def receive_thread():
    read_serial_and_save_list(ser1, data_list,save_path)

# 发送数据
def send_thread(ser1, x_begin ,y_begin ,x_step ,y_step ,x_end ,y_end ,x_judge ,y_judge):

        data = input('Input data to send:')
        if data == '1':
            # send_command_begin(ser1,60,20,1,1,120,160,0,0)
            send_command_begin(ser1, x_begin, y_begin, x_step, y_step, x_end, y_end, x_judge, y_judge)
        elif data == '2':
            send_command_stop(ser1)
        elif data == '3':   # 展示数组
            print(data_list)
        elif data == '4':   # 展示三维点云图
            pcdshow = o3d.geometry.PointCloud()
            data_list_temp =data_list
            pcdshow.points = o3d.utility.Vector3dVector(data_list_temp)
            o3d.visualization.draw_geometries([pcdshow])



if __name__ == '__main__':
    # 打开串口
    ser1 = serial.Serial('COM9', 115200, timeout=0)
    if ser1.isOpen():
        print("open success")
    else:
        print("open failed")

    data_list =[]
    save_path = "data/result1.pcb"


    # 启动线程
    receive_t = threading.Thread(target=receive_thread)
    send_t = threading.Thread(target=send_thread, args=(ser1, x_begin ,y_begin ,x_step ,y_step ,x_end ,y_end ,x_judge ,y_judge))

    receive_t.start()
    send_t.start()


