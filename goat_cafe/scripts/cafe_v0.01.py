#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped
import tkinter as tk
from std_msgs.msg import String
from threading import Timer,Event
import threading
import time
class CafeNavigation:
    def __init__(self, master):
        self.master = master
        master.title("Café Navigation")

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        self.home_position = (-2.999981, 1.000062, 0.0, 1.0)
        self.kitchen_position = (-0.030062, -8.721838, 0.0, 1.0)
        self.current_position = "home"  # Track current position

        self.order_queue = []  # To store incoming orders
        self.current_orders = []  # To store the current batch of orders (max 2)

        # Initialize robot to home position
        rospy.loginfo("Initializing to home position")
        self.set_initial_pose(self.home_position)

        # Create a frame for the order display
        self.order_frame = tk.Frame(master)
        self.order_frame.pack(pady=10)

        # Create labels for displaying orders
        self.order_labels = [tk.Label(self.order_frame, text="") for _ in range(2)]
        for label in self.order_labels:
            label.pack()

        # Create buttons
        self.confirm_button = tk.Button(master, text="Confirm Orders", command=self.confirm_orders)
        self.confirm_button.pack(pady=5)
        self.cancel_button = tk.Button(master, text="Cancel", command=self.cancel_order)
        self.cancel_button.pack(pady=5)

        self.confirmation_event = Event()
        self.table_timeout = 15  # seconds
        self.kitchen_wait_time = 15  # seconds

        # Disable buttons initially
        self.confirm_button.config(state=tk.DISABLED)
        self.cancel_button.config(state=tk.DISABLED)

        # ROS subscriber to handle incoming orders
        rospy.Subscriber('cafe_order', String, self.process_order)


    def set_initial_pose(self, position):
        initial_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
        rospy.sleep(1)  # Wait for the publisher to connect

        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = "map"
        initial_pose.header.stamp = rospy.Time.now()

        initial_pose.pose.pose.position.x = position[0]
        initial_pose.pose.pose.position.y = position[1]
        initial_pose.pose.pose.orientation.z = position[2]
        initial_pose.pose.pose.orientation.w = position[3]

        initial_pose.pose.covariance[0] = 0.25
        initial_pose.pose.covariance[7] = 0.25
        initial_pose.pose.covariance[35] = 0.06853892326654787

        initial_pose_pub.publish(initial_pose)
        rospy.loginfo("Initial pose set to home position.")

    def create_button(self, label, coordinates):
        button = tk.Button(self.master, text=label, command=lambda: self.move_to_goal(coordinates))
        button.pack()


    def move_to_goal(self, coordinates):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(Point(coordinates[0], coordinates[1], 0.0),
                                     Quaternion(0.0, 0.0, coordinates[2], coordinates[3]))
        self.client.send_goal(goal)
        self.client.wait_for_result()

    def return_to_home(self):
        rospy.loginfo("Returning to home position")
        self.move_to_goal(self.home_position)
        self.current_position = "home"
        self.update_order_display()

    def process_order(self, msg):
        rospy.loginfo(f"Received order: {msg.data}")
        self.order_queue.append(msg.data)
        self.update_order_display()
        if self.current_position != "kitchen":
            self.move_to_kitchen()

    
    def update_order_display(self):
        # Update the current_orders list
        while len(self.current_orders) < 2 and self.order_queue:
            self.current_orders.append(self.order_queue.pop(0))

        # Update the order labels
        for i, label in enumerate(self.order_labels):
            if i < len(self.current_orders):
                label.config(text=f"Order {i+1}: {self.current_orders[i]}")
            else:
                label.config(text="")

        # Enable/disable buttons based on whether there are orders and robot is in kitchen
        if self.current_orders and self.current_position == "kitchen":
            self.confirm_button.config(state=tk.NORMAL)
        else:
            self.confirm_button.config(state=tk.DISABLED)

        self.cancel_button.config(state=tk.NORMAL if self.current_orders else tk.DISABLED)

    def move_to_kitchen(self):
        rospy.loginfo("Moving to kitchen")
        self.move_to_goal(self.kitchen_position)
        self.current_position = "kitchen"
        self.update_order_display()

    def kitchen_confirmation_gui(self):
        # Ensure only one confirmation window is open
        if self.confirmation_window and self.confirmation_window.winfo_exists():
            self.confirmation_window.destroy()  # Close the existing window

        # Create a new window for confirmation
        self.confirmation_window = tk.Toplevel(self.master)
        self.confirmation_window.title("Order Confirmation")

        # Add a label for each of the first two orders
        order_text = "\n".join([f"Order {i + 1}: {order}" for i, order in enumerate(self.current_orders[:2])])
        label = tk.Label(self.confirmation_window, text=order_text)
        label.pack(pady=10)

        # Confirm button
        confirm_button = tk.Button(self.confirmation_window, text="Confirm Orders", command=self.confirm_orders)
        confirm_button.pack(side=tk.LEFT, padx=10, pady=10)

        # Cancel button
        cancel_button = tk.Button(self.confirmation_window, text="Cancel", command=self.cancel_order)
        cancel_button.pack(side=tk.RIGHT, padx=10, pady=10)

        # Start a timer for 1 minute to return to home if no confirmation
        if self.confirmation_timer:
            self.confirmation_timer.cancel()
        self.confirmation_timer = threading.Timer(60.0, self.return_to_home_if_no_confirmation)
        self.confirmation_timer.start()

    def confirm_orders(self):
        if self.current_position != "kitchen":
            rospy.logwarn("Cannot confirm orders: Robot is not in the kitchen")
            return

        for order in list(self.current_orders):  # Create a copy of the list to iterate over
            table_number = int(order.split(":")[0].split(" ")[1])
            self.move_to_table(table_number)
            if not self.wait_for_table_confirmation(table_number, order):
                # If confirmation fails (timeout), move back to kitchen and wait
                self.current_orders.remove(order)
                self.move_to_kitchen()
                rospy.loginfo(f"Waiting in kitchen for {self.kitchen_wait_time} seconds before resuming")
                time.sleep(self.kitchen_wait_time)
            else:
                self.current_orders.remove(order)

        if self.order_queue or self.current_orders:
            self.move_to_kitchen()
        else:
            self.return_to_home()

        self.update_order_display()


    def cancel_order(self):
        self.current_orders.clear()
        self.update_order_display()
        if not self.order_queue:
            self.return_to_home()

    def load_next_orders(self):
        # Move to the kitchen first
        self.move_to_goal(self.kitchen_position)

        # If current_orders is empty, load from order_queue
        if not self.current_orders:
            self.current_orders = self.order_queue[:2]
            self.order_queue = self.order_queue[2:]

        # Show the orders in the kitchen GUI
        self.kitchen_confirmation_gui()


    def return_to_home(self):
        rospy.loginfo("Returning to home position")
        self.move_to_goal(self.home_position)

    def move_to_table(self, table_number):
        positions = {
            1: (2.044646, -0.970785, 0.000000, 1.0),
            2: (2.044706, 1.567279, 0.0, 1.0),
            3: (-2.503098, 1.575518, 0.0, 1.0),
            4: (-2.503122, -0.978211, 0.0, 1.0)
        }
        position = positions.get(table_number)
        if position:
            self.move_to_goal(position)
            rospy.loginfo(f"Arrived at table {table_number}.")
        self.current_position = f"table_{table_number}"

    def wait_for_table_confirmation(self, table_number, order):
        self.confirmation_event.clear()
        confirmation_received = False
        
        def on_confirm():
            nonlocal confirmation_received
            confirmation_received = True
            self.confirmation_event.set()
            confirmation_window.destroy()

        def on_timeout():
            if not confirmation_received:
                rospy.logwarn(f"Confirmation timeout for table {table_number}")
                self.confirmation_event.set()
                confirmation_window.destroy()

        confirmation_window = tk.Toplevel(self.master)
        confirmation_window.title(f"Confirm Delivery - Table {table_number}")
        
        label = tk.Label(confirmation_window, text=f"Confirm delivery of order:\n{order}")
        label.pack(pady=10)
        
        confirm_button = tk.Button(confirmation_window, text="Confirm Delivery", command=on_confirm)
        confirm_button.pack(pady=10)
        
        timer = Timer(self.table_timeout, on_timeout)
        timer.start()
        
        self.master.wait_window(confirmation_window)
        self.confirmation_event.wait()
        
        timer.cancel()
        return confirmation_received

    def exit_application(self):
        rospy.loginfo("Exiting Café Navigation")
        self.master.quit()

if __name__ == '__main__':
    try:
        rospy.init_node('cafe_navigation_gui')
        root = tk.Tk()
        gui = CafeNavigation(root)
        root.mainloop()
    except rospy.ROSInterruptException:
        pass
