import tkinter as tk
import zmq

class ScoreApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Smart Basketball Hoop UI")
        
        self.root.geometry("500x300")  

        self.clear_button = tk.Button(root, text="New Session", command=self.clear_score, font=("Helvetica", 14))
        self.clear_button.pack(pady=10)

        self.clear_button = tk.Button(root, text="Save Results", command=self.save_score, font=("Helvetica", 14))
        self.clear_button.pack(pady=10)
    

    def clear_score(self):
        context = zmq.Context()

        socket = context.socket(zmq.REQ)
        socket.connect("tcp://localhost:5555")  

        request = "World"
        print(f"Sending request: {request}")
        socket.send_string(request)

        reply = socket.recv_string()
        print(f"Received reply: {reply}")

    def save_score(self):
        pass
        

root = tk.Tk()
app = ScoreApp(root)
root.mainloop()
