import zmq
import threading
import time

counter = 0
counter_lock = threading.Lock()

def handle_client_requests():
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind("tcp://*:5555")

    global counter
    while True:
        try:
            message = socket.recv_string()
            print(f"Received request: {message}")

            with counter_lock:
                counter = 0

            reply = f"Hello, {message}!"
            socket.send_string(reply)
        except zmq.ZMQError as e:
            print(f"ZMQ Error: {e}")
            break

client_thread = threading.Thread(target=handle_client_requests, daemon=True)
client_thread.start()

#your output code goes below. put this function and thread and counter
# and counter lock above your code

print("Server main task is running...")
while True:
    with counter_lock:
        counter += 1
        print(f"Background task: Doing main work, counter = {counter}")
    time.sleep(2)
