import rospy
import cv2
import numpy as np 
import apriltag
import Move_Filter as mf
import pandas as pd
import os
import datetime
from skimage.morphology import disk, opening, reconstruction
from getFeatures import getFeatures
from estimateAllTranslation import estimateAllTranslation
from applyGeometricTransformation import applyGeometricTransformation
from scipy.ndimage import maximum_filter
from feature_tracking.msg import label
from feature_tracking.msg import tag_t
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates


rospy.init_node('data_compering')
label_publish = rospy.Publisher('/label', label, queue_size=1)
tag_publish = rospy.Publisher('/tag', tag_t, queue_size=1)


def RGB2WB(cur_frame):
    kernel = np.ones((30, 30), np.uint8)
    dilated = cv2.dilate(cur_frame, kernel)
    no_dark_obj = np.where(dilated >= cur_frame, cur_frame, 0)

    # 使用 skimage 的 opening 方法去除小结构
    selem = disk(3)
    no_small_structures = opening(no_dark_obj.astype(np.uint8), selem)
    return no_small_structures


def objectTracking(rawVideo, draw_bb=False, play_realtime=False, save_to_file=False):
    # initilize
    # n_frame = 400
    n_frame = int(rawVideo.get(cv2.CAP_PROP_FRAME_COUNT))
    
    frames = np.empty((n_frame,),dtype=np.ndarray)
    frames_draw = np.empty((n_frame,),dtype=np.ndarray)
    bboxs = np.empty((n_frame,),dtype=np.ndarray)
    bboxs_t = np.empty((n_frame,),dtype=np.ndarray)

    for frame_idx in range(n_frame):
        _, frames[frame_idx] = rawVideo.read()

    # input label data in World Frame
    label_0_X = float (input("X Coordinate of label_0 in World Frame:"))
    label_0_Y = float (input("Y Coordinate of label_0 in World Frame:"))
    label_dis = float(input("Distance of label_0 and label_1 in X Coordinate of World Frame:"))


    # draw rectangle roi for target objects, or use default objects initilization
    if draw_bb:
        n_object = int(input("Number of label to track:"))
        bboxs[0] = np.empty((n_object,4,2), dtype=float)
        label_data = np.zeros((n_frame,n_object,2))
        target_data = np.zeros((n_frame,1,2))

        # 标注时请按照      0:left-top   1:right-top    2:right-bottom   3:left-bottom  便于后续视场角角度偏转纠正及相对位置计算
        for i in range(n_object):
            cv2.namedWindow("Select Object %d"%(i), cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Select Object %d"%(i), 1152, 648)
            cv2.moveWindow("Select Object %d"%(i), 0, 0)
            (xmin, ymin, boxw, boxh) = cv2.selectROI("Select Object %d"%(i),frames[0])
            cv2.destroyWindow("Select Object %d"%(i))
            label_data[0, i, :] = np.array([xmin+boxw/2,ymin+boxh/2]).reshape(1,2)
            bboxs[0][i,:,:] = np.array([[xmin,ymin],[xmin+boxw,ymin],[xmin,ymin+boxh],[xmin+boxw,ymin+boxh]]).astype(float)
        
        n_target = 1
        bboxs_t[0] = np.empty((n_target,4,2), dtype=float)
        for i in range(n_target):
            cv2.namedWindow("Select target %d"%(i), cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Select target %d"%(i), 1152, 648)
            cv2.moveWindow("Select target %d"%(i), 0, 0)
            (xmin, ymin, boxw, boxh) = cv2.selectROI("Select target %d"%(i),frames[0])
            cv2.destroyWindow("Select target %d"%(i))
            target_data[0, i, :] = np.array([xmin+boxw/2,ymin+boxh/2]).reshape(1,2)
            bboxs_t[0][i,:,:] = np.array([[xmin,ymin],[xmin+boxw,ymin],[xmin,ymin+boxh],[xmin+boxw,ymin+boxh]]).astype(float)
        gTracker = mf.Tracker(tracker_type="BOOSTING")
        gTracker.initWorking(frames[0],(xmin, ymin, boxw, boxh))
    else:
        n_object = 1
        bboxs[0] = np.array([[[291,187],[405,187],[291,267],[405,267]]]).astype(float)
    
    if save_to_file:
        out = cv2.VideoWriter('output.avi',0,cv2.VideoWriter_fourcc('M','J','P','G'),20.0,(frames[i].shape[1],frames[i].shape[0]))
    
    # Start from the first frame, do optical flow for every two consecutive frames.
    startXs,startYs = getFeatures(cv2.cvtColor(frames[0],cv2.COLOR_RGB2GRAY),bboxs[0],use_shi=False)
    # startXs,startYs = getFeatures(frames[0],bboxs[0],use_shi=False)
    frame_lost = 0
    tag_temp = tag_t()
    tag_temp.x = 0
    tag_temp.y = 0
    last_x = 0
    last_y = 0
    degree_tag = 0
    cx = 0
    cy = 0
    for i in range(1,n_frame):
        print('Processing Frame',i)
        newXs, newYs = estimateAllTranslation(startXs, startYs, frames[i-1], frames[i])
        Xs, Ys ,bboxs[i] = applyGeometricTransformation(startXs, startYs, newXs, newYs, bboxs[i-1])
        
        # update coordinates
        startXs = Xs
        startYs = Ys

        # update feature points as required
        n_features_left = np.sum(Xs!=-1)
        print('# of Features: %d'%n_features_left)
        if n_features_left < 25:#15
            print('Generate New Features')
            startXs,startYs = getFeatures(cv2.cvtColor(frames[i],cv2.COLOR_RGB2GRAY),bboxs[i],use_shi=False)

         #track the moving target
        _item = gTracker.track(frames[i])
        coord = _item.getMessage()['coord']
        # print(coord, coord[0],coord[0][0])
        target_data[i,0,:] = np.array([coord[0][0][0]/2+coord[0][1][0]/2,\
                                       coord[0][0][1]/2+coord[0][1][1]/2]).reshape(1,2)
        d_x = target_data[i,0,0] - last_x
        d_y = target_data[i,0,1] - last_y
        last_x = target_data[i,0,0]
        last_y = target_data[i,0,1]

        frames_draw[i] = frames[i].copy()
        for j in range(n_object):
            (xmin, ymin, boxw, boxh) = cv2.boundingRect(bboxs[i][j,:,:].astype(int))
            label_data[i,j,:] = np.array([xmin+boxw/2,ymin+boxh/2]).reshape(1,2)
            frames_draw[i] = cv2.rectangle(frames_draw[i], (xmin,ymin), (xmin+boxw,ymin+boxh), (255,0,0), 2)
            cv2.putText(frames_draw[i], str(j), (int(label_data[i,j,0]),int(label_data[i,j,1])), cv2.FONT_HERSHEY_COMPLEX, 2.0, (100, 200, 200), 8)
            # label_data[i,j,0]代表第i帧的第j个Lable的x, 1代表y
            # print('label_%d data: %d,%d'%(j,label_data[i,j,0],label_data[i,j,1]))
            for k in range(startXs.shape[0]):
                frames_draw[i] = cv2.circle(frames_draw[i], (int(startXs[k,j]),int(startYs[k,j])),3,(0,0,255),thickness=2)
        

        # calculate degree_cam
        degree_cam_rad = np.abs(np.arctan2(np.abs((label_data[i,0,1] - label_data[i,1,1])), np.abs((label_data[i,0,0] - label_data[i,1,0]))))
        degree_cam = degree_cam_rad * 180 / np.pi
        # print("degree_cam:%f"%degree_cam)

        # Apriltag detecting
        roi_x, roi_y, roi_w, roi_h = int(label_data[i,0,0]), int(label_data[i,0,1]), int(label_data[i,1,0]-label_data[i,0,0]), int(label_data[i,3,1]-label_data[i,0,1])
        roi_x, roi_y, roi_w, roi_h = int(coord[0][0][0]), int(coord[0][0][1]), int(coord[0][1][0]-coord[0][0][0]), int(coord[0][1][1]-coord[0][0][1])
        roi = frames[i][roi_y:roi_y+roi_h, roi_x:roi_x+roi_w]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        tags = at_detector.detect(gray)
        for tag in tags:
            #获取Apriltag角点及中心坐标
            tag.corners[0][0] += roi_x
            tag.corners[1][0] += roi_x
            tag.corners[2][0] += roi_x
            tag.corners[3][0] += roi_x
            tag.corners[0][1] += roi_y
            tag.corners[1][1] += roi_y
            tag.corners[2][1] += roi_y
            tag.corners[3][1] += roi_y
            a = tuple(tag.corners[0])   # left-top
            b = tuple(tag.corners[1])   # right-top
            c = tuple(tag.corners[2])   # right-bottom
            d = tuple(tag.corners[3])   # left-bottom
            cx = (tag.center[0] + roi_x)
            cy = (tag.center[1] + roi_y)
            #print(cx,cy)
            #绘制Apriltag4个角点
            frames_draw[i] = cv2.circle(frames_draw[i], tuple(tag.corners[0].astype(int)), 4, (255, 0, 0), 2)
            frames_draw[i] = cv2.circle(frames_draw[i], tuple(tag.corners[1].astype(int)), 4, (255, 0, 0), 2)
            frames_draw[i] = cv2.circle(frames_draw[i], tuple(tag.corners[2].astype(int)), 4, (255, 0, 0), 2)
            frames_draw[i] = cv2.circle(frames_draw[i], tuple(tag.corners[3].astype(int)), 4, (255, 0, 0), 2)
            #绘制Apriltag中心坐标
            (cX, cY) = (int(cx), int(cy))
            frames_draw[i] = cv2.circle(frames_draw[i], (cX, cY), 5, (255, 0, 255), -1)

            dx1 = d[0] - a[0]
            dy1 = d[1] - a[1]
            dx2 = c[0] - b[0]
            dy2 = c[1] - b[1] 
            degree1 = (np.arctan2(dx1, dy1)) *180/np.pi
            degree2 = (np.arctan2(dx2, dy2)) *180/np.pi
            degree_tag = (degree1 + degree2) / 2
            # print("degree_tag:%f"%degree_tag)
        
        if tags:
            tags
        # AprilTag无效时 使用KCF补偿
        else:
            cx += d_x
            cy += d_y
            # print("d_x:%d, d_y:%d"%(d_x,d_y))

            # 绘制Apriltag中心坐标z
            (cX, cY) = (int(cx), int(cy))
            frames_draw[i] = cv2.circle(frames_draw[i], (cX, cY), 5, (255, 0, 255), -1) 
            
            
        # calculate relateve degree & position        
        if label_data[i,0,1] < label_data[i,1,1]:
            degree = degree_tag - degree_cam
            w = np.abs(cx-label_data[i,0,0]) * np.cos(degree_cam_rad) + np.abs(cy-label_data[i,0,1]) * np.sin(degree_cam_rad)
            h = np.abs(cy-label_data[i,0,1]) * np.cos(degree_cam_rad) - np.abs(cx-label_data[i,0,0]) * np.sin(degree_cam_rad)
        else:
            degree = degree_tag + degree_cam
            w = np.abs(cx-label_data[i,0,0]) * np.cos(degree_cam_rad) - np.abs(cy-label_data[i,0,1]) * np.sin(degree_cam_rad)
            h = np.abs(cy-label_data[i,0,1]) * np.cos(degree_cam_rad) + np.abs(cx-label_data[i,0,0]) * np.sin(degree_cam_rad)
        print("degree:%f"%degree)

        # Translate Coordinate  Cam --> World
        K = label_dis/(label_data[i,1,0]-label_data[i,0,0])
        tag_X = label_0_X + w*K
        tag_Y = label_0_Y - h*K
        print("tag_pose_estimate  X:%f Y:%f"%(tag_X,tag_Y))

        # publish label & tag data
        label_temp = label()
        label_temp.x = label_data[i,0,0]
        label_temp.y = label_data[i,0,1]
        label_publish.publish(label_temp)
        
        tag_temp.x = cx
        tag_temp.y = cy
        tag_temp.X_e = tag_X
        tag_temp.Y_e = tag_Y
        tag_temp.angle = degree_tag
        tag_temp.rad = degree_tag * np.pi/180
        tag_publish.publish(tag_temp)

        # imshow if to play the result in real time
        if play_realtime:
            cv2.namedWindow("win", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("win", 1152, 648)
            cv2.moveWindow("win", 0, 0)
            cv2.imshow("win",frames_draw[i])
            cv2.waitKey(10)
        if save_to_file:
            out.write(frames_draw[i])

    if save_to_file:
        out.release()
        np.save("label_data.npy",label_data)
        np.save("target_data.npy",target_data)
    print(label_data.shape, target_data.shape)
    return label_data, target_data


if __name__ == "__main__":
    cap = cv2.VideoCapture("/home/junwei/catkin_ws/src/feature_tracking/experiment/real_world/0919_111.mp4")
    at_detector = apriltag.Detector(apriltag.DetectorOptions(families='tag36h11'))
    objectTracking(cap,draw_bb=True,play_realtime=True,save_to_file=True)
    cap.release()
   