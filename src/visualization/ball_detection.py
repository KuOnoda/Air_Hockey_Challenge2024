import cv2
import numpy as np
import time

class CameraTracker:
    def __init__(self, dev_num=-1, capture_time=5000000000, use_camera=True, search_range_devices=100, 
                 threshold_of_contour_length=100, frequency=60):
        # カメラデバイス番号がわからない場合はdev_numを -1 に設定。find_camera_device()で探索する。
        self.dev_num = dev_num
        self.capture_time = capture_time
        self.use_camera = use_camera
        self.search_range_devices = search_range_devices
        self.threshold_of_contour_length = threshold_of_contour_length
        self.frequency = frequency
        self.cap = None
        self.previous_cx = 0
        self.previous_cy = 0
    
    def find_camera_device(self):
        if self.dev_num == -1:
            for i in range(self.search_range_devices):
                cap = cv2.VideoCapture(i)
                if cap.isOpened():
                    self.dev_num = i
                    cap.release()
                    print(f'Camera device number is {self.dev_num}')
                    return
            print(f'Could not find a camera device within {self.search_range_devices} devices. '
                  'Please confirm the cable is correctly connected.')
            exit(1)

    def setup_camera(self):
        if self.use_camera:
            self.cap = cv2.VideoCapture(self.dev_num)
        else:
            self.cap = cv2.VideoCapture('./../data/air_hockey.mp4')
        self.cap.set(cv2.CAP_PROP_FPS, self.frequency)
    
    def track(self):
        start_time = time.time()
        while cv2.waitKey(int(1000 / self.frequency)) != 27:  # ESC to exit
            ret, frame = self.cap.read()
            if not ret:
                print("Failed to capture frame. Exiting...")
                break
            
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Red color mask
            lower_red1, upper_red1 = np.array([0, 100, 100]), np.array([10, 255, 255])
            lower_red2, upper_red2 = np.array([170, 100, 100]), np.array([180, 255, 255])
            mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask = mask1 + mask2
            res = cv2.bitwise_and(frame, frame, mask=mask)

            gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
            contours, _ = cv2.findContours(gray, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                if cv2.contourArea(contour) > self.threshold_of_contour_length:
                    moment = cv2.moments(contour)
                    if moment['m00'] != 0:
                        cx = int(moment['m10'] / moment['m00'])
                        cy = int(moment['m01'] / moment['m00'])
                        cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                        cv2.putText(frame, f'Center: ({cx}, {cy})', (cx + 10, cy - 10), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                        print(f'Center: ({cx}, {cy})')
                        print(f'Center velocity: ({(cx - self.previous_cx) * 1000 / self.frequency}, '
                              f'{(cy - self.previous_cy) * 1000 / self.frequency})')
                        self.previous_cx, self.previous_cy = cx, cy

            cv2.drawContours(frame, contours, -1, (0, 255, 0), 2)
            cv2.imshow(f'Video {self.frequency}Hz', frame)

            if time.time() - start_time > self.capture_time:
                break

    def release_resources(self):
        if self.cap:
            self.cap.release()
        cv2.destroyAllWindows()

# 使用例
if __name__ == "__main__":
    tracker = CameraTracker()
    tracker.find_camera_device()
    tracker.setup_camera()
    tracker.track()
    tracker.release_resources()
