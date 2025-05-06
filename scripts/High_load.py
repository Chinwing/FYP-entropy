#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy
from tkinter import Tk, Label, Button
from queue import Queue, Empty
import time
import threading
import sys
import random
import csv
import os

class Phase3GUI:
    def __init__(self):
        self.root = Tk()
        self.root.title("Phase 3: Reaction Test")
        self.root.configure(bg="azure")  
        
        self.buttons = []
        self.target_button = None
        self.selected_index = 0
        self.last_dpad = 0
        self.start_time = time.time()
        self.closed = False
        self.queue = Queue()

        for i in range(4):
            btn = Button(self.root, text=f"Button {i+1}", font=("Helvetica", 18), width=15,
                        bg="paleturquoise1")
            btn.grid(row=0, column=i, padx=10, pady=10)
            self.buttons.append(btn)

        self.feedback = Label(self.root, text="", font=("Helvetica", 16), bg="azure")
        self.feedback.grid(row=1, column=0, columnspan=4)

        self.timer_label = Label(self.root, text="Time Remaining: 5:00", 
                                 font=("Helvetica", 18), bg="azure")
        self.timer_label.grid(row=2, column=0, columnspan=4, pady=10)

        log_dir = "/home/chinwing/catkin_ws/src/FYP/result_csv"
        os.makedirs(log_dir, exist_ok=True)  
        
        self.reaction_log = open(
            os.path.join(log_dir, "phase3_reaction_log.csv"),
            "a", 
            newline=''
        )
        self.csv_writer = csv.writer(self.reaction_log)
        self.csv_writer.writerow(["timestamp", "target_button", "clicked_button", "reaction_time", "correct"])

        self.root.after(100, self.process_queue)
        self.highlight_selected()
        
        rospy.Subscriber("/phase_status", Float32, self.phase_callback)
        rospy.Subscriber("/phase_timer", Float32, self.timer_callback)
        rospy.Subscriber("/joy", Joy, self.joy_callback)
        
        self.schedule_next_flash()

    def highlight_selected(self):
        for i, btn in enumerate(self.buttons):
            if i == self.selected_index:
                btn.config(relief="sunken", bg="deepskyblue")
            else:
                btn.config(relief="raised", bg="paleturquoise1")
                if i == self.target_button:
                    btn.config(bg="purple")

    def joy_callback(self, msg):
        if self.closed:
            return
        
        dpad = msg.axes[6]
        if dpad == -1 and self.last_dpad != -1:  
            self.selected_index = min(len(self.buttons) - 1, self.selected_index + 1)
            self.highlight_selected()
        elif dpad == 1 and self.last_dpad != 1: 
            self.selected_index = max(0, self.selected_index - 1)
            self.highlight_selected()
        
        self.last_dpad = dpad

        if msg.buttons[3] == 1: 
            self.button_clicked(self.selected_index)

    def schedule_next_flash(self):
        if not self.closed:
            delay = random.randint(5, 10) * 1000
            self.root.after(delay, self.flash_random_button)

    def flash_random_button(self):
        if self.closed:
            return
        if self.target_button is not None:
            self.buttons[self.target_button].config(bg="paleturquoise1")

        self.target_button = random.randint(0, 3)
        self.buttons[self.target_button].config(bg="purple")
        self.start_time = time.time()

    def button_clicked(self, idx):
        if self.target_button is None or self.closed:
            return

        reaction_time = round(time.time() - self.start_time, 3)
        correct = idx == self.target_button
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")

        self.csv_writer.writerow([timestamp, self.target_button + 1, idx + 1, reaction_time, correct])
        self.reaction_log.flush()

        if correct:
            self.feedback.config(text=f"Correct! Time: {reaction_time}s", fg="forestgreen")
            self.buttons[self.target_button].config(bg="paleturquoise1")
            self.target_button = None
            self.schedule_next_flash()
        else:
            self.feedback.config(text=f"Wrong! Time: {reaction_time}s", fg="red")
            self.buttons[idx].config(bg="red")
            self.root.after(500, self.reset_after_incorrect)

    def reset_after_incorrect(self):
        if self.closed:
            return
        if self.target_button is not None:
            self.buttons[self.target_button].config(bg="purple")
        self.highlight_selected()

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
        if msg.data != 3:
            rospy.loginfo("Phase 3 ended, closing GUI...")
            self.queue.put("CLOSE")

    def process_queue(self):
        try:
            msg = self.queue.get_nowait()
            if msg == "CLOSE":
                self.closed = True
                self.reaction_log.close()
                self.root.after(0, self.shutdown)
        except Empty:
            pass
        if not self.closed:
            self.root.after(100, self.process_queue)

    def shutdown(self):
        self.root.quit()
        self.root.destroy()

def launch_gui():
    app = Phase3GUI()
    app.root.mainloop()

if __name__ == '__main__':
    rospy.init_node('phase3_gui', anonymous=True)
    gui_thread = threading.Thread(target=launch_gui)
    gui_thread.daemon = True
    try:
        gui_thread.start()
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
    except rospy.ROSInterruptException:
        pass
    finally:
        sys.exit(0)
