# -*- coding: utf-8 -*-

# This script is used to subscribe image topic and turn it to frame_img

# Use instructions to transform image to video :    ffmpeg -threads 2 -y -r 30 -i frame_%04d.jpg raw_video.mp4

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class VideoRecorder:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/iris/camera/image_raw', Image, self.image_callback)
        self.frames = []
        self.recording = False
 
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            if self.recording:
                self.frames.append(cv_image)
        except Exception as e:
            print(e)
 
    def start_recording(self):
        self.frames = []
        self.recording = True
 
    def stop_recording(self):
        self.recording = False
        if self.frames:
            self.save_frames()

 
    def save_frames(self):
        for i, frame in enumerate(self.frames):
            filename = 'frame_{:04d}.jpg'.format(i)
            cv2.imwrite(filename, frame)
            #self.out = cv2.VideoWriter('/home/junwei/catkin_ws/src/featrue_tracking/scripts/test_raw.avi', cv2.VideoWriter_fourcc('M','J','P','G'), 30, (frame.shape[1],frame.shape[0]))
            #self.out.write(frame)  # 将帧写入视频文件
        print('Saved {} frames.'.format(len(self.frames)))
 
if __name__ == '__main__':
    rospy.init_node('video_recorder_node', anonymous=True)
    recorder = VideoRecorder()
 
    try:
        while not rospy.is_shutdown():
            cmd = input("Enter 'start' to begin recording or 'stop' to stop recording: ")
            if cmd == 'start':
                recorder.start_recording()
            elif cmd == 'stop':
                recorder.stop_recording()
            
    except rospy.ROSInterruptException:
        pass