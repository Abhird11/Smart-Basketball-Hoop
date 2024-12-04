import tkinter as tk
import zmq
import socket

class ScoreApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Smart Basketball Hoop UI")
        
        self.root.geometry("500x300")  

        self.clear_button = tk.Button(root, text="New Session", command=self.clear_score, font=("Helvetica", 14))
        self.clear_button.pack(pady=10)

        self.clear_button = tk.Button(root, text="Save Results", command=self.save_score, font=("Helvetica", 14))
        self.clear_button.pack(pady=10)
    

    def is_server_running(self, host="localhost", port=5555):
        try:
            with socket.create_connection((host, port), timeout=2):
                return True
        except (socket.timeout, ConnectionRefusedError):
            return False

    def clear_score(self):
        if not self.is_server_running():
            print("Server is not running.")
            return

        context = zmq.Context()

        socket = context.socket(zmq.REQ)
        try:
            socket.connect("tcp://localhost:5555")

            request = "World"
            print(f"Sending request: {request}")
            socket.send_string(request)

            socket.setsockopt(zmq.RCVTIMEO, 2000) 

            try:
                reply = socket.recv_string()
                print(f"Received reply: {reply}")
            except zmq.Again:
                print("No response from server, request timed out.")
        except zmq.ZMQError as e:
            print("Error connecting to server:", e)
        finally:
            socket.close()
        

    def save_score(self):
        pass
        

root = tk.Tk()
app = ScoreApp(root)
root.mainloop()
