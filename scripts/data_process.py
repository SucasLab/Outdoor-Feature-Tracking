from objectTracking import objectTracking
import cv2
import numpy as np
def data_process(label_data,target_data):
    x_label = int(input("Length of label x:"))
    y_label = int(input("Length of label y:"))
    label_data_last = label_data[0,:,:]
    data_real = np.zeros(target_data.shape)
    for i in range(target_data.shape[0]):#将图像坐标转化为基于0路标的坐标系
        target_data[i,0,0] = target_data[i,0,0] - label_data[i,0,0]
        target_data[i,0,1] = -target_data[i,0,1] + label_data[i,0,1]
        label_data[i,:,0] = label_data[i,:,0] - label_data[i,0,0]
        label_data[i,:,1] = -label_data[i,:,1] + label_data[i,0,1]
    for i in range(label_data.shape[0]):
        pixel2m_x = 2*x_label/(abs(label_data[i,0,0]-label_data[i,1,0])\
                        +abs(label_data[i,2,0]-label_data[i,3,0]))
        pixel2m_y = 2*y_label/(abs(label_data[i,0,1]-label_data[i,3,1])\
                        +abs(label_data[i,1,1]-label_data[i,2,1]))
        delta = np.array([np.mean(label_data[i,:,0]-label_data_last[:,0]),\
                          np.mean(label_data[i,:,1]-label_data_last[:,1])])
        label_data_last = label_data[i,:,:]
        data_real[i,:,:] = (target_data[i,:,:] - delta)*np.array([pixel2m_x,pixel2m_y])
        print(data_real[i,:,:])   

if __name__ == "__main__":
    label_data = np.load("label_data.npy")
    target_data = np.load("target_data.npy")
    data_process(label_data,target_data)
    # cap = cv2.VideoCapture("F:/Projects/KLT-Feature-Tracking-master/DJI_0013.avi")
    # label_data, target_data = objectTracking(cap,draw_bb=True,play_realtime=True,save_to_file=True)
    # cap.release()