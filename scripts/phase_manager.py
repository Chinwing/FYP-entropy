#!/usr/bin/env python3

import rospy
import csv
import os
import threading
import sys
import signal
from threading import Event
from std_msgs.msg import Float32, String
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from datetime import datetime

class PhaseManager:
    def __init__(self):
        self.prev_joy_button_state = 0
        self.joy_pressed = False

        rospy.init_node('phase_manager', anonymous=True)

        self.phase = 0
        self.lap_count = 0
        self.lap_start_time = None

        rospy.loginfo("Entropy logging paused.")
        self.timer_start = None
        self.timer_limit = 360

        self.last_pos = None
        self.current_pos = None
        self.spawn_crossed_once = False
        self.lap_in_progress = False
        self.phase0_gui_launched = False

        self.start_x = 4.55
        self.spawn_x = 4.63
        self.y_range = (-3.90, -3.00)

        self.phase_pub = rospy.Publisher("/phase_status", Float32, queue_size=1)
        self.entropy_trigger_pub = rospy.Publisher("/start_entropy", Float32, queue_size=1)
        self.timer_pub = rospy.Publisher("/phase_timer", Float32, queue_size=1)

        self.csv_files = {
            0: open("/home/chinwing/catkin_ws/src/FYP/result_csv/phase0_log.csv", "a", newline=''),
            1: open("/home/chinwing/catkin_ws/src/FYP/result_csv/phase1_log.csv", "a", newline=''),
            2: open("/home/chinwing/catkin_ws/src/FYP/result_csv/phase2_log.csv", "a", newline=''),
            3: open("/home/chinwing/catkin_ws/src/FYP/result_csv/phase3_log.csv", "a", newline=''),
        }
        self.csv_writers = {phase: csv.writer(f) for phase, f in self.csv_files.items()}
        for writer in self.csv_writers.values():
            writer.writerow(["timestamp", "phase", "phase_time (mm:ss)", "entropy_total", "entropy_ang", 
                            "entropy_lin", "est_error_ang", "est_error_lin"])

        self.entropy_triggered = False
        self.entropy_total = 0.0
        self.entropy_ang = 0.0
        self.entropy_lin = 0.0
        self.est_ang = 0.0
        self.est_lin = 0.0
        self.lap_info_pub = rospy.Publisher("/lap_info", String, queue_size=1)

        self.mid_gui_launched = False
        self.high_gui_launched = False

        self.gui_path = "/home/chinwing/catkin_ws/src/FYP/behaviour_detection_package/scripts"

        self.y_pressed_phase1 = False
        self.y_pressed_phase2 = False
        self.y_pressed_phase3 = False

        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/entropy", Float32, self.entropy_callback)
        rospy.Subscriber("/entropy_ang", Float32, self.entropy_ang_callback)
        rospy.Subscriber("/entropy_lin", Float32, self.entropy_lin_callback)
        rospy.Subscriber("/estimation_error_ang", Float32, self.est_ang_callback)
        rospy.Subscriber("/estimation_error_lin", Float32, self.est_lin_callback)
        rospy.Subscriber("/joy", Joy, self.joy_callback)

        rospy.Timer(rospy.Duration(1.0), self.log_data)
        rospy.Timer(rospy.Duration(1.0), self.check_timer)

    def joy_callback(self, data):
        current_state = data.buttons[3]
        if current_state == 1 and self.prev_joy_button_state == 0:
            if self.phase in [0.5, 1.5, 2.5]:
                self.joy_pressed = True
                rospy.loginfo(f"Y button pressed in transition phase {self.phase}")
            else:
                rospy.loginfo(f"Y button ignored in phase {self.phase}")
        self.prev_joy_button_state = current_state

    def crossed_line(self, x_thresh, y_range):
        if self.last_pos is None or self.current_pos is None:
            return False
        last_x, last_y = self.last_pos
        curr_x, curr_y = self.current_pos
        y_min, y_max = y_range
        
        return (y_min <= curr_y <= y_max and 
                last_x > x_thresh and curr_x <= x_thresh and 
                last_x > curr_x)  

    def crossed_spawn_line(self):
        if self.last_pos is None or self.current_pos is None:
            return False
        last_x, last_y = self.last_pos
        curr_x, curr_y = self.current_pos
        y_min, y_max = self.y_range
        
        return (y_min <= curr_y <= y_max and 
                last_x > self.spawn_x and curr_x <= self.spawn_x and 
                last_x > curr_x)  

    def in_spawn_zone(self, pos, tol=0.3):
        return abs(pos[0] - self.spawn_x) <= tol and self.y_range[0] <= pos[1] <= self.y_range[1]

    def odom_callback(self, data):
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        self.last_pos = self.current_pos
        self.current_pos = (x, y)

        if self.crossed_line(self.start_x, self.y_range) and self.phase == 0:
            self.lap_start_time = rospy.Time.now()
            self.lap_in_progress = True
            rospy.loginfo(f"Lap {self.lap_count + 1} started.")
            self.lap_info_pub.publish(f"Lap {self.lap_count + 1} started.")
            if self.lap_count == 1:
                self.entropy_triggered = True
                self.entropy_trigger_pub.publish(1.0)
        
        if self.phase == 0 and not self.phase0_gui_launched:
            phase0_path = os.path.join(self.gui_path, "trial.py")
            threading.Thread(target=lambda: os.system(f"python3 {phase0_path}")).start()
            self.phase0_gui_launched = True

        if self.lap_in_progress and self.in_spawn_zone(self.current_pos) and self.phase == 0 and self.lap_start_time and (rospy.Time.now() - self.lap_start_time).to_sec() > 10.0:
            self.lap_count += 1
            elapsed = (rospy.Time.now() - self.lap_start_time).to_sec()
            self.lap_info_pub.publish(f"Lap {self.lap_count} completed. Time: {elapsed:.2f}s")
            rospy.loginfo(f"Lap {self.lap_count} ended. Duration: {elapsed:.2f} seconds")
            self.lap_in_progress = False

            if self.lap_count == 2:
                rospy.loginfo("Lap 2 completed.")
                self.entropy_triggered = False
                self.entropy_trigger_pub.publish(0.0)
                self.phase = 0.5
                self.joy_pressed = False
                self.lap_count = 0
                self.spawn_crossed_once = False

        if self.phase == 0.5 and self.joy_pressed:
            self.y_pressed_phase1 = True
            self.joy_pressed = False
            rospy.loginfo("Y button pressed. Please cross the start line to begin Phase 1.")

        if self.phase == 0.5 and self.y_pressed_phase1 and self.crossed_line(self.start_x, self.y_range):
            self.phase = 1
            self.phase_pub.publish(self.phase)
            rospy.loginfo("Phase 1 started.")
            self.entropy_triggered = True
            self.entropy_trigger_pub.publish(1.0)
            self.timer_start = rospy.Time.now()
            self.y_pressed_phase1 = False
            low_load_path = os.path.join(self.gui_path, "Low_load.py")
            threading.Thread(target=lambda: os.system(f"python3 {low_load_path}")).start()

        if self.phase == 1.5 and self.joy_pressed:
            self.y_pressed_phase2 = True
            self.joy_pressed = False
            rospy.loginfo("Y button pressed. Please cross the start line to begin Phase 2.")

        if self.phase == 1.5 and self.y_pressed_phase2 and self.crossed_line(self.start_x, self.y_range):
            self.phase = 2
            self.phase_pub.publish(self.phase)
            rospy.loginfo("Phase 2 started.")
            self.entropy_triggered = True
            self.entropy_trigger_pub.publish(1.0)
            self.timer_start = rospy.Time.now()
            self.y_pressed_phase2 = False
            if not self.mid_gui_launched:
                mid_load_path = os.path.join(self.gui_path, "Mid_load.py")
                threading.Thread(target=lambda: os.system(f"python3 {mid_load_path}")).start()
                self.mid_gui_launched = True

        if self.phase == 2.5 and self.joy_pressed:
            self.y_pressed_phase3 = True
            self.joy_pressed = False
            rospy.loginfo("Y button pressed. Please cross the start line to begin Phase 3.")

        if self.phase == 2.5 and self.y_pressed_phase3 and self.crossed_line(self.start_x, self.y_range):
            self.phase = 3
            self.phase_pub.publish(self.phase)
            rospy.loginfo("Phase 3 started.")
            self.entropy_triggered = True
            self.entropy_trigger_pub.publish(1.0)
            self.timer_start = rospy.Time.now()
            self.y_pressed_phase3 = False
            if not self.high_gui_launched:
                high_load_path = os.path.join(self.gui_path, "High_load.py")
                threading.Thread(target=lambda: os.system(f"python3 {high_load_path}")).start()
                self.high_gui_launched = True

        if self.phase == 1 and self.crossed_spawn_line():
            self.entropy_trigger_pub.publish(0.0)
            self.entropy_triggered = False
            self.phase = 1.5
            self.phase_pub.publish(self.phase)
            rospy.loginfo("Phase 1 ended. Awaiting Y button to continue.")
            self.timer_start = None

        if self.phase == 2 and self.crossed_spawn_line():
            self.entropy_trigger_pub.publish(0.0)
            self.entropy_triggered = False
            self.phase = 2.5
            self.phase_pub.publish(self.phase)
            rospy.loginfo("Phase 2 ended. Awaiting Y button to continue.")
            self.timer_start = None

        if self.phase == 3 and self.crossed_line(self.start_x, self.y_range) and rospy.Time.now() - self.timer_start > rospy.Duration(5):
            self.entropy_trigger_pub.publish(0.0)
            self.entropy_triggered = False
            self.phase = 3.5
            self.phase_pub.publish(self.phase)
            rospy.loginfo("Phase 3 ended.")
            self.timer_start = None

    def check_timer(self, event):
        if self.timer_start and int(self.phase) in [1, 2, 3]:
            elapsed = (rospy.Time.now() - self.timer_start).to_sec()
            remaining = max(0, self.timer_limit - elapsed)
            minutes = int(remaining // 60)
            seconds = int(remaining % 60)
            self.timer_pub.publish(remaining)
            if remaining <= 0:
                self.phase += 0.5
                self.phase_pub.publish(self.phase)
                self.entropy_triggered = False
                self.entropy_trigger_pub.publish(0.0)
                self.timer_start = None

    def entropy_callback(self, msg):
        self.entropy_total = msg.data

    def entropy_ang_callback(self, msg):
        self.entropy_ang = msg.data

    def entropy_lin_callback(self, msg):
        self.entropy_lin = msg.data

    def est_ang_callback(self, msg):
        self.est_ang = msg.data

    def est_lin_callback(self, msg):
        self.est_lin = msg.data

    def log_data(self, event):
        if self.timer_start:
            elapsed = (rospy.Time.now() - self.timer_start).to_sec()
            minutes = int(elapsed // 60)
            seconds = int(elapsed % 60)
            elapsed_str = f"{minutes:02}:{seconds:02}"
        else:
            elapsed_str = "00:00"

        now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        if int(self.phase) in self.csv_writers and (self.phase % 1.0 == 0.0) and self.entropy_triggered:
            writer = self.csv_writers[int(self.phase)]
            writer.writerow([
                now,
                f"Phase {self.phase}",
                elapsed_str,
                round(self.entropy_total, 5),
                round(self.entropy_ang, 5),
                round(self.entropy_lin, 5),
                round(self.est_ang, 5),
                round(self.est_lin, 5)
            ])
            self.csv_files[int(self.phase)].flush()

    def shutdown(self):
        for f in self.csv_files.values():
            f.close()
        
       
        if hasattr(self, 'log_timer'):
            self.log_timer.shutdown()
        if hasattr(self, 'check_timer'):
            self.check_timer.shutdown()

        rospy.loginfo("Closing GUI applications...")
        os.system("pkill -f phase0_GUI.py")
        os.system("pkill -f Low_load.py")
        os.system("pkill -f Mid_load.py")
        os.system("pkill -f High_load.py")
        os.system("pkill -f trial.py")
        rospy.sleep(0.5)

        nodes_to_kill = [
            "amcl",
            "map_server",
            "rviz",
            "entropy_calc_node", 
            "joy_node",
            "teleop_twist_joy",
            "estimation_error_node",
            "robot_state_publisher"
        ]
        
        for node in nodes_to_kill:
            rospy.loginfo(f"Shutting down /{node}...")
            os.system(f"rosnode kill /{node}")
            rospy.sleep(0.3)

        rospy.loginfo("Shutting down Gazebo...")
        os.system("pkill -SIGINT gzclient")
        rospy.sleep(1.0)
        os.system("pkill -SIGINT gzserver")
        rospy.sleep(1.0)

        # Final cleanup
        rospy.loginfo("Final cleanup...")
        os.system("pkill -SIGINT rosmaster")
        rospy.sleep(0.5)

if __name__ == '__main__':
    node = PhaseManager()
    rospy.spin()