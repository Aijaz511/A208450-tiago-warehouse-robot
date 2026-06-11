#!/usr/bin/env python3

import rospy
import yaml
import math
import threading
import tkinter as tk
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion, PoseWithCovarianceStamped
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import String
import tf.transformations
import rospkg

STATUS_COLORS = {
    "IDLE":       "#5c6bc0",
    "NAVIGATING": "#f9a825",
    "SUCCEEDED":  "#43a047",
    "FAILED":     "#e53935",
    "CANCELLED":  "#757575",
}

class TiagoNavUI:
    def __init__(self, root):
        self.root = root
        self.root.title("TIAGo Navigation Control Panel")
        self.root.geometry("480x680")
        self.root.configure(bg="#1e1e2e")
        self.root.resizable(False, False)

        rospy.init_node("tiago_nav_ui", anonymous=True)
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base.")

        rp = rospkg.RosPack()
        pkg_path = rp.get_path("tiago_nav_ui")
        with open(f"{pkg_path}/config/waypoints.yaml", "r") as f:
            data = yaml.safe_load(f)
        self.waypoints = data["waypoints"]

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.pose_callback)
        rospy.Subscriber("/aruco_scan_result", String, self._scan_callback)

        self.status = "IDLE"
        self.current_goal_name = None

        self._build_ui()
        self._start_ros_spin()

    def _build_ui(self):
        title_font  = ("Helvetica", 16, "bold")
        label_font  = ("Helvetica", 10)
        btn_font    = ("Helvetica", 11, "bold")
        status_font = ("Helvetica", 12, "bold")

        header = tk.Frame(self.root, bg="#12121f", pady=12)
        header.pack(fill="x")
        tk.Label(header, text="TIAGo Navigation Panel",
                 font=title_font, bg="#12121f", fg="#cdd6f4").pack()

        self.status_frame = tk.Frame(self.root, bg="#1e1e2e", pady=6)
        self.status_frame.pack(fill="x", padx=20)

        self.status_indicator = tk.Label(
            self.status_frame, text="[IDLE]",
            font=status_font, bg="#1e1e2e", fg=STATUS_COLORS["IDLE"])
        self.status_indicator.pack(side="left")

        self.goal_label = tk.Label(
            self.status_frame, text="",
            font=label_font, bg="#1e1e2e", fg="#a6adc8")
        self.goal_label.pack(side="left", padx=10)

        pose_frame = tk.LabelFrame(
            self.root, text=" Robot Pose ",
            font=label_font, bg="#1e1e2e", fg="#a6adc8",
            bd=1, relief="groove", padx=10, pady=6)
        pose_frame.pack(fill="x", padx=20, pady=(4, 8))

        self.pose_var = tk.StringVar(value="X: --   Y: --   Yaw: --")
        tk.Label(pose_frame, textvariable=self.pose_var,
                 font=label_font, bg="#1e1e2e", fg="#cdd6f4").pack()

        wp_frame = tk.LabelFrame(
            self.root, text=" Select Destination ",
            font=label_font, bg="#1e1e2e", fg="#a6adc8",
            bd=1, relief="groove", padx=14, pady=10)
        wp_frame.pack(fill="both", expand=True, padx=20, pady=(0, 8))

        self.wp_buttons = {}
        for name, data in self.waypoints.items():
            desc = data.get("description", "")
            frm = tk.Frame(wp_frame, bg="#1e1e2e", pady=4)
            frm.pack(fill="x")

            btn = tk.Button(
                frm,
                text=f">> {name.replace('_', ' ')}",
                font=btn_font,
                bg="#313244", fg="#cdd6f4",
                activebackground="#45475a", activeforeground="#ffffff",
                relief="flat", bd=0, padx=12, pady=10, cursor="hand2",
                command=lambda n=name: self.send_goal(n))
            btn.pack(side="left", fill="x", expand=True)

            tk.Label(frm, text=desc, font=label_font,
                     bg="#1e1e2e", fg="#6c7086", width=18, anchor="w").pack(side="left")

            self.wp_buttons[name] = btn

        self.cancel_btn = tk.Button(
            self.root, text="[X] Cancel Navigation",
            font=btn_font,
            bg="#45202a", fg="#f38ba8",
            activebackground="#5c2333", activeforeground="#ffffff",
            relief="flat", bd=0, pady=10, cursor="hand2",
            command=self.cancel_goal)
        self.cancel_btn.pack(fill="x", padx=20, pady=(0, 6))

        log_frame = tk.LabelFrame(
            self.root, text=" Log ",
            font=label_font, bg="#1e1e2e", fg="#a6adc8",
            bd=1, relief="groove")
        log_frame.pack(fill="x", padx=20, pady=(0, 6))

        self.log_box = tk.Text(
            log_frame, height=4, bg="#12121f", fg="#a6e3a1",
            font=("Courier", 9), relief="flat", state="disabled",
            insertbackground="#cdd6f4")
        self.log_box.pack(fill="x", padx=4, pady=4)

        scan_frame = tk.LabelFrame(
            self.root, text=" Last Scan Result ",
            font=label_font, bg="#1e1e2e", fg="#a6adc8",
            bd=1, relief="groove")
        scan_frame.pack(fill="x", padx=20, pady=(0, 12))

        self.scan_var = tk.StringVar(value="No scan yet")
        tk.Label(scan_frame, textvariable=self.scan_var,
                 font=("Courier", 9), bg="#12121f", fg="#cba6f7",
                 wraplength=420, anchor="w", justify="left").pack(
                     fill="x", padx=4, pady=4)

    def pose_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_yaw = math.degrees(yaw)
        self.root.after(0, self._update_pose_display)

    def _update_pose_display(self):
        self.pose_var.set(
            f"X: {self.current_x:.2f}   Y: {self.current_y:.2f}   "
            f"Yaw: {self.current_yaw:.1f} deg")

    def _scan_callback(self, msg):
        self.root.after(0, lambda: self.scan_var.set(msg.data))
        self._log(f"Scan: {msg.data}")

    def send_goal(self, name):
        wp = self.waypoints[name]
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = wp["x"]
        goal.target_pose.pose.position.y = wp["y"]

        q = tf.transformations.quaternion_from_euler(0, 0, wp["yaw"])
        goal.target_pose.pose.orientation = Quaternion(*q)

        self.current_goal_name = name
        self._set_status("NAVIGATING", f"-> {name.replace('_', ' ')}")
        self._log(f"Sending goal: {name} ({wp['x']:.2f}, {wp['y']:.2f})")
        self._set_buttons_state("disabled")
        self.client.send_goal(goal, done_cb=self._done_callback)

    def cancel_goal(self):
        self.client.cancel_all_goals()
        self._set_status("CANCELLED")
        self._log("Navigation cancelled by user.")
        self._set_buttons_state("normal")

    def _done_callback(self, state, result):
        if state == GoalStatus.SUCCEEDED:
            self._set_status("SUCCEEDED", f"Reached {self.current_goal_name.replace('_', ' ')}")
            self._log(f"Reached: {self.current_goal_name}")
        else:
            self._set_status("FAILED", "Goal failed or aborted")
            self._log(f"Failed to reach: {self.current_goal_name} (state={state})")
        self.root.after(0, lambda: self._set_buttons_state("normal"))

    def _set_status(self, status, detail=""):
        self.status = status
        color = STATUS_COLORS.get(status, "#ffffff")
        self.root.after(0, lambda: self.status_indicator.config(
            text=f"[{status}]", fg=color))
        self.root.after(0, lambda: self.goal_label.config(text=detail))

    def _set_buttons_state(self, state):
        for btn in self.wp_buttons.values():
            btn.config(state=state)

    def _log(self, msg):
        def _write():
            self.log_box.config(state="normal")
            self.log_box.insert("end", f"[{rospy.Time.now().to_sec():.1f}] {msg}\n")
            self.log_box.see("end")
            self.log_box.config(state="disabled")
        self.root.after(0, _write)

    def _start_ros_spin(self):
        threading.Thread(target=rospy.spin, daemon=True).start()

if __name__ == "__main__":
    root = tk.Tk()
    app = TiagoNavUI(root)
    root.mainloop()
