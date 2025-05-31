#!/usr/bin/env python3

import rospy
from std_msgs.msg import UInt32 # Using UInt32 for item ID

class VendingMachineROSNode:
    def __init__(self):
        rospy.init_node('vending_machine_ros_node', anonymous=True)

        # Subscribe to a topic that will signal an item to be dispensed
        self.dispense_subscriber = rospy.Subscriber(
            '/dispense_item',
            UInt32,
            self.dispense_callback
        )

        rospy.loginfo("Vending Machine ROS Node initialized and waiting for dispense commands.")
        rospy.loginfo("To simulate dispensing, publish an item ID (UInt32) to /dispense_item")
        rospy.loginfo("Example: rostopic pub /dispense_item std_msgs/UInt32 'data: 1'")

    def dispense_callback(self, msg):
        item_id = msg.data
        rospy.loginfo(f"Received command to dispense item ID: {item_id}")

        # In a real robot, this is where you would trigger the dispensing mechanism.
        # For this simulation, we'll just log the action.
        rospy.loginfo(f"SIMULATION: Dispensing item ID: {item_id}...")
        # Simulate some processing time
        rospy.sleep(1)
        rospy.loginfo(f"SIMULATION: Item ID: {item_id} dispensed.")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        vending_node = VendingMachineROSNode()
        vending_node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Vending Machine ROS Node shutting down.")
    except Exception as e:
        rospy.logerr(f"An error occurred in Vending Machine ROS Node: {e}")
