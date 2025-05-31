#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, LaserScan, Imu
from nav_msgs.msg import Odometry
# from blockchain_oracle import BlockchainOracle # This will be created later
import cv2
from cv_bridge import CvBridge
import numpy as np
import json
import hashlib # Added for calculate_hash

# Placeholder for BlockchainOracle until it's defined
class BlockchainOracle:
    def __init__(self):
        rospy.loginfo("BlockchainOracle initialized (placeholder)")

    def submit_sensor_data(self, metadata):
        rospy.loginfo(f"Submitting to Blockchain (placeholder): {json.dumps(metadata)}")

class SensorDataTracker:
    def __init__(self):
        rospy.init_node('sensor_blockchain_tracker')

        # Inicializar ponte blockchain
        self.oracle = BlockchainOracle()
        self.bridge = CvBridge()

        # Configurar subscribers
        rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        rospy.Subscriber('/odom', Odometry, self.odometry_callback)

        # Parâmetros de configuração
        self.track_images = rospy.get_param('~track_images', True)
        self.image_interval = rospy.get_param('~image_interval', 5.0) # seconds
        self.last_image_time = rospy.get_time() # Initialize with current time

    def image_callback(self, msg):
        """Processa e registra dados de imagem"""
        current_time = rospy.get_time()

        if self.track_images and (current_time - self.last_image_time > self.image_interval):
            try:
                # Converter imagem ROS para OpenCV
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

                # Extrair características da imagem
                image_features = self.extract_image_features(cv_image)

                # Criar metadados para blockchain
                metadata = {
                    'sensor_type': 'camera',
                    'timestamp': current_time,
                    'image_hash': self.calculate_image_hash(cv_image),
                    'resolution': f"{msg.width}x{msg.height}",
                    'features_detected': len(image_features) if image_features else 0
                }

                # Submeter para blockchain via oracle
                self.oracle.submit_sensor_data(metadata)
                self.last_image_time = current_time
                rospy.loginfo(f"Image data submitted. Features: {metadata['features_detected']}")

            except Exception as e:
                rospy.logerr(f"Erro ao processar imagem: {e}")

    def lidar_callback(self, msg):
        """Registra dados do LiDAR"""
        current_time = rospy.get_time()
        lidar_data = {
            'sensor_type': 'lidar',
            'timestamp': current_time,
            'range_min': msg.range_min,
            'range_max': msg.range_max,
            'angle_min': msg.angle_min,
            'angle_max': msg.angle_max,
            'num_readings': len(msg.ranges),
            'data_hash': self.calculate_hash(str(msg.ranges))
        }

        self.oracle.submit_sensor_data(lidar_data)
        rospy.loginfo("LiDAR data submitted.")

    def imu_callback(self, msg):
        """Registra dados do IMU"""
        current_time = rospy.get_time()
        imu_data = {
            'sensor_type': 'imu',
            'timestamp': current_time,
            'orientation': {
                'x': msg.orientation.x,
                'y': msg.orientation.y,
                'z': msg.orientation.z,
                'w': msg.orientation.w
            },
            'angular_velocity': {
                'x': msg.angular_velocity.x,
                'y': msg.angular_velocity.y,
                'z': msg.angular_velocity.z
            },
            'linear_acceleration': {
                'x': msg.linear_acceleration.x,
                'y': msg.linear_acceleration.y,
                'z': msg.linear_acceleration.z
            },
            'data_hash': self.calculate_hash(str(msg)) # Hash of the whole message
        }

        self.oracle.submit_sensor_data(imu_data)
        rospy.loginfo("IMU data submitted.")

    def odometry_callback(self, msg):
        """Registra dados de Odometria"""
        current_time = rospy.get_time()
        odometry_data = {
            'sensor_type': 'odometry',
            'timestamp': current_time,
            'position': {
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y,
                'z': msg.pose.pose.position.z,
            },
            'orientation': {
                'x': msg.pose.pose.orientation.x,
                'y': msg.pose.pose.orientation.y,
                'z': msg.pose.pose.orientation.z,
                'w': msg.pose.pose.orientation.w,
            },
            'data_hash': self.calculate_hash(str(msg)) # Hash of the whole message
        }
        self.oracle.submit_sensor_data(odometry_data)
        rospy.loginfo("Odometry data submitted.")

    def extract_image_features(self, image):
        """Extrai características visuais usando OpenCV"""
        try:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            # SIFT is patented, using ORB instead as it's free
            orb = cv2.ORB_create()
            keypoints, descriptors = orb.detectAndCompute(gray, None)
            return keypoints
        except Exception as e:
            rospy.logerr(f"Error extracting image features: {e}")
            return None

    def calculate_image_hash(self, image):
        """Calcula hash perceptual da imagem (dHash)"""
        try:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            # Resize to 9x8 for dHash calculation (8x8 differences)
            resized = cv2.resize(gray, (9, 8), interpolation=cv2.INTER_AREA)
            # Calculate differences between adjacent pixels
            diff = resized[:, 1:] > resized[:, :-1]
            return ''.join(['1' if bit else '0' for bit in diff.flatten()])
        except Exception as e:
            rospy.logerr(f"Error calculating image hash: {e}")
            return ""

    def calculate_hash(self, data_string):
        """Calcula hash SHA-256 de dados"""
        return hashlib.sha256(data_string.encode()).hexdigest()

if __name__ == '__main__':
    try:
        tracker = SensorDataTracker()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Sensor Blockchain Tracker node terminated.")
    except Exception as e:
        rospy.logerr(f"An error occurred in SensorDataTracker: {e}")
