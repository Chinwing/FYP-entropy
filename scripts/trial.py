#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Float32
from tkinter import Tk, Label
from datetime import datetime
from queue import Queue, Empty
import threading

# This script is the GUI for Phase 0 of the project.
class Phase0GUI:
    def __init__(self):
        self.root = Tk()
        self.root.title("Phase 0: Lap Tracker")
        self.root.configure(bg="CadetBlue1")

        self.lap_label = Label(self.root, text="Lap: 0", font=("Helvetica", 20), bg="CadetBlue1", fg="black")
        self.lap_label.pack(pady=10)

        self.time_label = Label(self.root, text="Elapsed Time: 00:00", font=("Helvetica", 18), bg="CadetBlue1", fg="black")
        self.time_label.pack(pady=10)

        self.start_time = None
        self.lap_time = None  
        self.current_lap = 0 
        self.phase_complete = False
        self.closed = False
        self.queue = Queue()
        self.root.after(100, self.update_timer)
        self.root.after(100, self.process_queue)

        rospy.Subscriber("/lap_info", String, self.lap_callback)
        rospy.Subscriber("/phase_status", Float32, self.phase_callback)

    def lap_callback(self, msg):
        if self.closed:
            return
        self.lap_label.config(text=msg.data)
        
        if "started" in msg.data:
            lap_num = int(msg.data.split()[1])
            self.current_lap = lap_num
            self.lap_time = datetime.now() 
            rospy.loginfo(f"Timer reset for Lap {lap_num}")
            
            if lap_num == 1 and self.start_time is None:
                self.start_time = datetime.now()
                rospy.loginfo("Overall timer started with Lap 1")
        
        elif "completed" in msg.data:
            self.lap_time = None
            if "Lap 2" in msg.data:
                self.phase_complete = True
                self.queue.put("CLOSE") 

    def phase_callback(self, msg):
        if msg.data != 0 and not self.closed:
            self.queue.put("CLOSE")

    def update_timer(self):
        if not self.closed:
            if self.lap_time is not None:
                elapsed = datetime.now() - self.lap_time
                minutes, seconds = divmod(int(elapsed.total_seconds()), 60)
                self.time_label.config(text=f"Lap {self.current_lap} Time: {minutes:02}:{seconds:02}")
            elif self.current_lap == 0:
                self.time_label.config(text="Waiting to start...")
            else:
                self.time_label.config(text=f"Waiting for Lap {self.current_lap + 1}")
            self.root.after(1000, self.update_timer)

    def process_queue(self):
        try:
            msg = self.queue.get_nowait()
            if msg == "CLOSE":
                self.closed = True
                self.root.quit()
                self.root.destroy()
        except Empty:
            pass
        if not self.closed:
            self.root.after(100, self.process_queue)

def launch_gui():
    app = Phase0GUI()
    app.root.mainloop()

if __name__ == '__main__':
    rospy.init_node('phase0_gui', anonymous=True)
    threading.Thread(target=launch_gui).start()
    rospy.spin()
