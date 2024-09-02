#!/usr/bin/env python

import rospy
import tkinter as tk
from std_msgs.msg import String
from threading import Timer

class CafeOrderClient:
    def __init__(self, master, table_number):
        self.master = master
        master.title(f"Table {table_number} - Order Menu")

        self.table_number = table_number
        self.order = []

        # Create a simple menu with 3 items
        self.create_menu_item("Coffee")
        self.create_menu_item("Tea")
        self.create_menu_item("Sandwich")

        # Confirm order button
        confirm_button = tk.Button(master, text="Confirm Order", command=self.confirm_order)
        confirm_button.pack()

        # ROS publisher to send the order
        self.pub = rospy.Publisher('cafe_order', String, queue_size=10)

    def create_menu_item(self, item_name):
        var = tk.StringVar(value="0")
        check = tk.Checkbutton(self.master, text=item_name, variable=var, onvalue=item_name, offvalue="0",
                               command=lambda: self.update_order(item_name, var))
        check.pack()

    def update_order(self, item_name, var):
        if var.get() == "0":
            self.order.remove(item_name)
        else:
            self.order.append(item_name)

    def confirm_order(self):
        if self.order:
            order_str = f"Table {self.table_number}: " + ", ".join(self.order)
            rospy.loginfo(f"Order confirmed: {order_str}")
            self.pub.publish(order_str)
            self.master.quit()

if __name__ == '__main__':
    rospy.init_node('cafe_order_client')

    # Assume the table number is passed as a ROS parameter
    table_number = rospy.get_param('~table_number', 1)

    root = tk.Tk()
    gui = CafeOrderClient(root, table_number)
    root.mainloop()
