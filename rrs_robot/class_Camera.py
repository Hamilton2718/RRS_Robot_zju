from picamera2 import Picamera2
import cv2
import numpy as np
import threading

class Camera:
    def __init__(self, width, height):
        self.picam2 = Picamera2()
        self.width = width
        self.height = height
        self.config = self.picam2.create_video_configuration(
            main={"format": 'XRGB8888', "size": (self.width, self.height)},
            controls={
                "FrameDurationLimits": (8333, 8333),
                "ExposureTime": 8000
            }
        )
        self.picam2.configure(self.config)
        self.lower_pink = np.array([140, 150, 50])
        self.upper_pink = np.array([180, 255, 255])
        self.picam2.start()

    def take_pic(self):
        try:
            image = self.picam2.capture_array()
            if image is None or image.size == 0:
                print("take_pic: 捕获到空图像")
                return None
            return image
        except Exception as e:
            print(f"take_pic: 相机捕获异常: {str(e)}")
            return None

    def show_video(self, image):
        cv2.imshow("Live", image)
        cv2.waitKey(1)

    def find_ball(self, image):
        image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(image_hsv, self.lower_pink, self.upper_pink)
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            (x, y), radius = cv2.minEnclosingCircle(largest_contour)
            area = cv2.contourArea(largest_contour)
            if area > 200:
                cv2.circle(image, (int(x), int(y)), int(radius), (0, 255, 0), 2)
                self.show_video(image)
                d = radius * 2
                h = 10000 / d

                x -= self.width / 2
                y -= self.height / 2
                return int(x), int(y), int(area)
        self.show_video(image)
        return -1, -1, 0

    def clean_up_cam(self):
        self.picam2.stop()
        self.picam2.close()
        cv2.destroyAllWindows()
