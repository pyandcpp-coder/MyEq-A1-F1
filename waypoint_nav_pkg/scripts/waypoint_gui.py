#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Empty
import tkinter as tk
from tkinter import ttk
import threading

WAYPOINT_NAMES = [
    'station_a', 'station_b', 'station_c', 'station_d', 
    'station_e', 'station_f', 'docking', 'home'
]

class WaypointGUI(Node):
    def __init__(self):
        super().__init__('waypoint_gui_node')
        
        self.single_waypoint_pub = self.create_publisher(String, 'go_to_single_waypoint', 10)
        self.sequence_pub = self.create_publisher(String, 'go_to_waypoint_sequence', 10)
        self.cancel_pub = self.create_publisher(Empty, 'cancel_navigation', 10)
        
        self.create_subscription(String, 'navigation_status', self.status_callback, 10)
        
        self.root = tk.Tk()
        self.root.title("Waypoint Navigation GUI")
        
        self.waypoint_vars = {}
        
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        waypoints_frame = ttk.LabelFrame(main_frame, text="Select Waypoints")
        waypoints_frame.grid(row=0, column=0, columnspan=2, padx=5, pady=5, sticky="ew")
        
        row, col = 0, 0
        for name in WAYPOINT_NAMES:
            var = tk.BooleanVar()
            cb = ttk.Checkbutton(waypoints_frame, text=name, variable=var)
            cb.grid(row=row, column=col, padx=5, pady=2, sticky="w")
            self.waypoint_vars[name] = var
            col += 1
            if col > 3:
                col = 0
                row += 1
                
        controls_frame = ttk.Frame(main_frame)
        controls_frame.grid(row=1, column=0, padx=5, pady=10, sticky="ew")
        
        self.go_single_button = ttk.Button(controls_frame, text="Go to First Selected (and Home)", command=self.go_to_single)
        self.go_single_button.grid(row=0, column=0, padx=5)
        
        self.go_sequence_button = ttk.Button(controls_frame, text="Run Full Sequence (and Home)", command=self.go_to_sequence)
        self.go_sequence_button.grid(row=0, column=1, padx=5)
        
        self.cancel_button = ttk.Button(controls_frame, text="CANCEL Navigation", command=self.cancel_navigation)
        self.cancel_button.grid(row=0, column=2, padx=20)
        
        status_frame = ttk.LabelFrame(main_frame, text="Status")
        status_frame.grid(row=2, column=0, columnspan=2, padx=5, pady=5, sticky="ew")
        
        self.status_label_var = tk.StringVar(value="Idle. Select waypoint(s) and press Go.")
        self.status_label = ttk.Label(status_frame, textvariable=self.status_label_var, wraplength=400)
        self.status_label.grid(row=0, column=0, padx=5, pady=5, sticky="w")
        
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
    def get_selected_waypoints(self):
        selected = []
        for name, var in self.waypoint_vars.items():
            if var.get():
                selected.append(name)
        return selected

    def go_to_single(self):
        selected = self.get_selected_waypoints()
        if not selected:
            self.status_label_var.set("Error: No waypoint selected.")
            return
        
        first_waypoint = selected[0]
        self.status_label_var.set(f"Sending single goal: {first_waypoint}")
        self.single_waypoint_pub.publish(String(data=first_waypoint))

    def go_to_sequence(self):
        selected = self.get_selected_waypoints()
        if not selected:
            self.status_label_var.set("Error: No waypoints selected for sequence.")
            return
            
        sequence_str = ",".join(selected)
        self.status_label_var.set(f"Sending sequence: {sequence_str}")
        self.sequence_pub.publish(String(data=sequence_str))

    def cancel_navigation(self):
        self.status_label_var.set("Sending CANCEL request...")
        self.cancel_pub.publish(Empty())

    def status_callback(self, msg):
        self.status_label_var.set(msg.data)
        
    def on_closing(self):
        self.get_logger().info("GUI window closed, shutting down node.")
        self.root.destroy()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    gui_node = WaypointGUI()
    
    spin_thread = threading.Thread(target=rclpy.spin, args=(gui_node,), daemon=True)
    spin_thread.start()
    
    try:
        gui_node.root.mainloop()
    except KeyboardInterrupt:
        pass
    
    gui_node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    spin_thread.join()

if __name__ == '__main__':
    main()