#!/usr/bin/env python3
import rospy
import subprocess
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time

class CameraPublisher:
    def __init__(self):
        rospy.init_node('camera_publisher', anonymous=True)
        self.image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
        self.bridge = CvBridge()
        
        self.camera_device = rospy.get_param('~camera_device', '/dev/video0')
        self.frame_rate = rospy.get_param('~frame_rate', 30)
        self.resolution = rospy.get_param('~resolution', '640x480')
        self.enable_display = rospy.get_param('~enable_display', False)
        self.enable_save = rospy.get_param('~enable_save', False)
        
        self.video_writer = None
        if self.enable_save:
            self.setup_video_writer()
            
    def setup_video_writer(self):
        width, height = map(int, self.resolution.split('x'))
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"camera_video_{timestamp}.avi"
        self.video_writer = cv2.VideoWriter(filename, fourcc, self.frame_rate, (width, height))
        rospy.loginfo(f"视频将保存为: {filename}")
    
    def start_camera_stream(self):
        ffmpeg_cmd = [
            'ffmpeg',
            '-f', 'v4l2',
            '-input_format', 'mjpeg',
            '-video_size', self.resolution,
            '-framerate', str(self.frame_rate),
            '-i', self.camera_device,
            '-f', 'image2pipe',
            '-pix_fmt', 'bgr24',
            '-vcodec', 'rawvideo',
            '-'
        ]
        
        try:
            self.ffmpeg_process = subprocess.Popen(
                ffmpeg_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=10**8
            )
            rospy.loginfo("FFmpeg相机流启动成功")
            return True
        except Exception as e:
            rospy.logerr(f"启动FFmpeg失败: {e}")
            return False
    
    def run(self):
        if not self.start_camera_stream():
            return
            
        width, height = map(int, self.resolution.split('x'))
        frame_size = width * height * 3
        
        try:
            while not rospy.is_shutdown():
                raw_frame = self.ffmpeg_process.stdout.read(frame_size)
                if len(raw_frame) != frame_size:
                    continue
                
                frame = np.frombuffer(raw_frame, dtype=np.uint8).reshape((height, width, 3))
                
                ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                ros_image.header.stamp = rospy.Time.now()
                ros_image.header.frame_id = "camera_frame"
                self.image_pub.publish(ros_image)
                
                if self.enable_display:
                    cv2.imshow('Camera Feed', frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break
                
                if self.enable_save and self.video_writer:
                    self.video_writer.write(frame)
                    
        except Exception as e:
            rospy.logerr(f"处理帧数据时出错: {e}")
        finally:
            self.cleanup()
    
    def cleanup(self):
        if hasattr(self, 'ffmpeg_process') and self.ffmpeg_process:
            self.ffmpeg_process.terminate()
            self.ffmpeg_process.wait()
        
        if self.video_writer:
            self.video_writer.release()
        
        if self.enable_display:
            cv2.destroyAllWindows()

def main():
    publisher = CameraPublisher()
    publisher.run()

if __name__ == '__main__':
    main()
