import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImageSender(Node):

    def __init__(self):
        super().__init__('image_sender')
        self.publisher_ = self.create_publisher(Image, 'image', 10)
        self.br = CvBridge()

        # Timestamps uzantısını yorum olarak belirleyin
        self.timestamp_extension = '.png'  # Kullanılan uzantıyı buraya ekleyin
        self.timestamp_file = '/home/rasp7/image/timestamps.txt'  # Kendi kaydettiğiniz yolu ayarlayın
        self.image_folder = '/home/rasp7/image'  # Kendi kaydettiğiniz yolu ayarlayın

        # Timestamps'ları yükleme işlemi
        self.load_timestamps()

        self.current_index = 0
        self.rate = 30  # FPS
        self.timer = self.create_timer(1.0 / self.rate, self.timer_callback)

    def load_timestamps(self):
        self.get_logger().info(f'Checking for timestamp file at: {self.timestamp_file}')
        if os.path.exists(self.timestamp_file):
            self.get_logger().info('Timestamp file found. Reading...')
            with open(self.timestamp_file, 'r') as f:
                self.timestamps = [line.strip() for line in f.readlines()]
            self.get_logger().info(f'Loaded {len(self.timestamps)} timestamps.')
        else:
            self.get_logger().error('Timestamp file not found!')
            self.timestamps = []

    def timer_callback(self):
        if self.current_index < len(self.timestamps):
            timestamp = self.timestamps[self.current_index]
            img_path = os.path.join(self.image_folder, f'{timestamp}{self.timestamp_extension}')  # Uzantıyı burada ekleyin

            self.get_logger().info(f'Looking for image at: {img_path}')
            if os.path.exists(img_path):
                img = cv2.imread(img_path)
                if img is not None:
                    img_msg = self.br.cv2_to_imgmsg(img, "bgr8")
                    self.publisher_.publish(img_msg)
                    self.get_logger().info(f'Published image: {img_path}')
                else:
                    self.get_logger().warning(f'Could not read image: {img_path}')
            else:
                self.get_logger().warning(f'Image not found: {img_path}')

            self.current_index += 1
        else:
            self.get_logger().info('All images have been published.')
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ImageSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
