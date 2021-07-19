import time
import zmq
import signal
import threading

#exit handler function
def exit_handler(a,b):
    print(a)
    print(b)

def senderThread():
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind("tcp://10.34.34.26:9001")

    data = {
        "test" : "test"
    }
    
    while True:
        print(data)
        socket.send_json(data)
        time.sleep(0.5)
        
    

def recvThread():

    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.connect("tcp://10.34.34.104:9000")

    socket.setsockopt(zmq.SUBSCRIBE, b'')

    #data = socket.recv_json()
    #start_index = data["seq_number"]
    #start = time.perf_counter()
    cs_hist = []
    l_hist = []

    while True:
        data = socket.recv_json()
        if data["type"] == "carState":
            cs_hist.append(data)
            
        
        elif data["type"] == "location":
            l_hist.append(data)
            print(data)

    
    #end = time.perf_counter()
    #print((end-start)/(5000-start_index))

def main():
    thread1 = threading.Thread(None, recvThread)
    thread2 = threading.Thread(None, senderThread)
    
    thread1.daemon = True
    thread2.daemon = True
    thread1.start()
    thread2.start()

    while True:
        time.sleep(1)
    
    

if __name__ == "__main__":
    main()
