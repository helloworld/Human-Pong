from socketIO_client import SocketIO, LoggingNamespace
import random 
import time



def send_data(socket):
	value1 = 0.5
	value2 = 0.5
	while True:
		value1 += random.uniform(-0.01, 0.01)
		value2 += random.uniform(-0.05, 0.05)
		socket.emit('data', 1, value1)
		socket.emit('data', 2, value2)
		time.sleep(0.1)
		# socket.wait(seconds=0.5)


try:
    socket = SocketIO('localhost', 3000, wait_for_connection=False)
    send_data(socket)
except ConnectionError:
    print('The server is down. Try again later.')



