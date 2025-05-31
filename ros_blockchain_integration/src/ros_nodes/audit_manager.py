#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
# from blockchain_bridge import FabricBridge # This will be created/defined later
import json
import hashlib

# Placeholder for FabricBridge until it's defined
class FabricBridge:
    def __init__(self):
        rospy.loginfo("FabricBridge initialized (placeholder)")

    def submit_transaction(self, function_name, data_json):
        rospy.loginfo(f"Submitting to Fabric (placeholder): func='{function_name}', data='{data_json}'")

class RobotAuditNode:
    def __init__(self):
        rospy.init_node('robot_audit_blockchain')
        self.fabric_bridge = FabricBridge()

        # Subscribers para dados do robô
        rospy.Subscriber('/joint_states', JointState, self.joint_callback, queue_size=10)
        rospy.Subscriber('/cmd_vel', Twist, self.velocity_callback, queue_size=10)

        # Configurações de auditoria
        self.audit_frequency = rospy.get_param('~audit_freq', 1.0) # Hz
        self.robot_id = rospy.get_param('~robot_id', 'robot_001')
        self.last_joint_audit_time = rospy.get_time()
        self.last_cmd_vel_audit_time = rospy.get_time()

    def joint_callback(self, msg):
        """Registra estados das juntas na blockchain com base na frequência"""
        current_time = rospy.get_time()
        if (1.0 / self.audit_frequency) > (current_time - self.last_joint_audit_time):
            return

        try:
            audit_data = {
                'robot_id': self.robot_id,
                'timestamp': current_time,
                'joint_names': list(msg.name),
                'joint_positions': list(msg.position),
                'joint_velocities': list(msg.velocity) if msg.velocity else [], # Ensure velocity exists
                'joint_efforts': list(msg.effort) if msg.effort else [],       # Ensure effort exists
                'data_hash': self.calculate_hash(str(msg))
            }

            self.fabric_bridge.submit_transaction(
                'RecordJointState',
                json.dumps(audit_data)
            )
            self.last_joint_audit_time = current_time
            rospy.loginfo("Joint state audit data submitted.")
        except Exception as e:
            rospy.logerr(f"Error in joint_callback: {e}")

    def velocity_callback(self, msg):
        """Auditoria de comandos de velocidade com base na frequência"""
        current_time = rospy.get_time()
        if (1.0 / self.audit_frequency) > (current_time - self.last_cmd_vel_audit_time):
            return

        try:
            command_data = {
                'robot_id': self.robot_id,
                'timestamp': current_time,
                'linear_velocity': {'x': msg.linear.x, 'y': msg.linear.y, 'z': msg.linear.z},
                'angular_velocity': {'x': msg.angular.x, 'y': msg.angular.y, 'z': msg.angular.z},
                'command_hash': self.calculate_hash(str(msg))
            }

            self.fabric_bridge.submit_transaction(
                'RecordVelocityCommand',
                json.dumps(command_data)
            )
            self.last_cmd_vel_audit_time = current_time
            rospy.loginfo("Velocity command audit data submitted.")
        except Exception as e:
            rospy.logerr(f"Error in velocity_callback: {e}")

    def calculate_hash(self, data_string):
        """Calcula hash criptográfico dos dados"""
        return hashlib.sha256(data_string.encode('utf-8')).hexdigest()

if __name__ == '__main__':
    try:
        audit_node = RobotAuditNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Robot Audit Blockchain node terminated.")
    except Exception as e:
        rospy.logerr(f"An error occurred in RobotAuditNode: {e}")
