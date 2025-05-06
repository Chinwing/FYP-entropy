#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from tkinter import Tk, Label
from queue import Queue, Empty
import threading

class Phase2GUI:
    def __init__(self):
        self.root = Tk()
        self.root.title("Phase 2: Countdown & Timer")
        self.root.configure(bg="lavender")  

        self.countdown_label = Label(
            self.root, 
            text="Countdown: 500 - 3", 
            font=("Helvetica", 24),
            bg="lavender",  
            fg="navy" 
        )
        self.countdown_label.pack(pady=10)

        self.timer_label = Label(
            self.root, 
            text="Time Remaining: 5:00", 
            font=("Helvetica", 18),
            bg="lavender",  
            fg="navy"   
        )
        self.timer_label.pack(pady=10)

        self.closed = False
        self.queue = Queue()
        self.root.after(100, self.process_queue)

        rospy.Subscriber("/phase_status", Float32, self.phase_callback)
        rospy.Subscriber("/phase_timer", Float32, self.timer_callback)

    def timer_callback(self, msg):
        if self.closed:
            return
        remaining = msg.data
        minutes = int(remaining // 60)
        seconds = int(remaining % 60)
        self.timer_label.config(text=f"Time Remaining: {minutes}:{seconds:02d}")
        if remaining <= 0:
            self.timer_label.config(text="Time's up!")

    def phase_callback(self, msg):
        if self.closed:
            return
        if msg.data != 2:  
            rospy.loginfo("Phase 2 ended, closing GUI...")
            self.queue.put("CLOSE")

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
    app = Phase2GUI()
    app.root.mainloop()

if __name__ == '__main__':
    rospy.init_node('phase2_gui', anonymous=True)
    threading.Thread(target=launch_gui).start()
    rospy.spin()
