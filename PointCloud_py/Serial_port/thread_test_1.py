import threading
import time

def test():
    for i in range(5):
        print(threading.current_thread().name+' test ',i)
        time.sleep(1)


thread = threading.Thread(target=test,name='TestThread')
thread.start()
thread.join()

for i in range(5):
    print(threading.current_thread().name+' main ', i)
    print(thread.name+' is alive ', thread.is_alive())
    time.sleep(1)
