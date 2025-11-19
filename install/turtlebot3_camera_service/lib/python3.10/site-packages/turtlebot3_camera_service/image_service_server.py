import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from ament_index_python.packages import get_package_share_directory

# 생성한 서비스 정의 파일 import
from turtlebot3_camera_service_interfaces.srv import ImageCapture


class ImageServiceServer(Node):
    def __init__(self):
        super().__init__('image_service_server')
        
        # 1. 서비스 서버 생성
        # Service: /image_capture, Type: ImageCapture
        self.srv = self.create_service(ImageCapture, 'image_capture', self.capture_image_callback)
        self.get_logger().info('Image Capture Service Server Ready.')

        # 2. 이미지 토픽 구독
        # Topic: /TurtleBot3Burger/front_camera/image_color (과제 요구사항)
        self.subscription = self.create_subscription(
            Image,
            '/TurtleBot3Burger/front_camera/image_color',
            self.image_callback,
            1  # 큐 사이즈
        )
        self.latest_image_msg = None  # 가장 최근에 받은 이미지 메시지를 저장할 변수
        self.bridge = CvBridge()
        
        # 3. 이미지 저장 경로 설정
        # 현재 패키지 공유 디렉토리 아래 'images' 폴더에 저장
        self.save_dir = os.path.join(get_package_share_directory('turtlebot3_camera_service'), 'images')
        os.makedirs(self.save_dir, exist_ok=True)
        self.get_logger().info(f'Images will be saved in: {self.save_dir}')

    def image_callback(self, msg):
        """카메라 토픽이 들어올 때마다 최신 이미지 메시지를 업데이트."""
        self.latest_image_msg = msg

    def capture_image_callback(self, request, response):
        """서비스 요청이 들어오면 이미지를 저장하고 응답."""
        if self.latest_image_msg is None:
            response.success = False
            response.message = 'Error: No image data received yet.'
            self.get_logger().warn('Service requested, but no image data available.')
            return response

        try:
            # ROS Image 메시지를 OpenCV(CV) 형식으로 변환
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image_msg, 'bgr8')
            
            # 저장 경로와 파일 이름 설정
            file_path = os.path.join(self.save_dir, request.filename)
            
            # 이미지 저장
            if cv2.imwrite(file_path, cv_image):
                response.success = True
                response.message = f'Image successfully saved to {file_path}'
                self.get_logger().info(f'Success: Image saved as {request.filename}')
            else:
                response.success = False
                response.message = f'Error: Could not save image to {file_path}'
                self.get_logger().error(f'Failed to save image: {file_path}')

        except Exception as e:
            response.success = False
            response.message = f'Internal Error: {str(e)}'
            self.get_logger().error(f'Exception occurred: {e}')
        
        return response

def main(args=None):
    rclpy.init(args=args)
    image_service_server = ImageServiceServer()
    try:
        rclpy.spin(image_service_server)
    except KeyboardInterrupt:
        pass
    finally:
        image_service_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()