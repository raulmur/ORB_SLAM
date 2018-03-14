#!/usr/bin/env python
print "importing libraries"

import rosbag
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
import ImageFile
import time, sys, os
import argparse
import cv2
import numpy as np
import csv
import math

#structure
# dataset/cam0/TIMESTAMP.png
# dataset/camN/TIMESTAMP.png
# dataset/imu.csv

def parseArgs():
    #setup the argument list
    parser = argparse.ArgumentParser(description='Extract a ROS bag containing a image and imu topics.')
    parser.add_argument('--folder',  metavar='folder', nargs='?', help='Data folder')
    parser.add_argument('--output-bag', metavar='output_bag',  default="output.bag", help='ROS bag file %(default)s')
    parser.add_argument('--video',  metavar='video_file', nargs='?', help='Video filename')
    parser.add_argument('--imu',  metavar='imu_file', nargs='?', help='Imu filename')
    parser.add_argument('--video-time-offset',  metavar='video_time_offset', type=float, default=0.0, help='The time of the first video frame based on the Imu clock (default: %(default)s)', required=False)
    parser.add_argument('--video-from-to', metavar='video_from_to', type=float, nargs=2, help='Use the video frames starting from up to this time [s] based on the video clock.')

    #print help if no argument is specified
    if len(sys.argv)<2:
        parser.print_help()
        sys.exit(1)

    #parse the args
    parsed = parser.parse_args()
    return parsed

def getImageFilesFromDir(dir):
    '''Generates a list of files from the directory'''
    image_files = list()
    timestamps = list()
    if os.path.exists(dir):
        for path, names, files in os.walk(dir):
            for f in files:
                if os.path.splitext(f)[1] in ['.bmp', '.png', '.jpg']:
                    image_files.append( os.path.join( path, f ) )
                    timestamps.append(os.path.splitext(f)[0]) 
    #sort by timestamp
    sort_list = sorted(zip(timestamps, image_files))
    image_files = [file[1] for file in sort_list]
    return image_files

def getCamFoldersFromDir(dir):
    '''Generates a list of all folders that start with cam e.g. cam0'''
    cam_folders = list()
    if os.path.exists(dir):
        for path, folders, files in os.walk(dir):
            for folder in folders:                
                if folder[0:3] == "cam":
                    cam_folders.append(folder)
    return cam_folders

def getImuCsvFiles(dir):
    '''Generates a list of all csv files that start with imu'''
    imu_files = list()
    if os.path.exists(dir):
        for path, folders, files in os.walk(dir):
            for file in files:
                if file[0:3] == 'imu' and os.path.splitext(file)[1] == ".csv":
                    imu_files.append( os.path.join( path, file ) )
    
    return imu_files

def loadImageToRosMsg(filename):
    image_np = cv2.imread(filename, cv2.IMREAD_GRAYSCALE)
    
    timestamp_nsecs = os.path.splitext(os.path.basename(filename))[0]
    timestamp = rospy.Time( secs=int(timestamp_nsecs[0:-9]), nsecs=int(timestamp_nsecs[-9:]) )

    rosimage = Image()
    rosimage.header.stamp = timestamp
    rosimage.height = image_np.shape[0]
    rosimage.width = image_np.shape[1]
    rosimage.step = rosimage.width  #only with mono8! (step = width * byteperpixel * numChannels)
    rosimage.encoding = "mono8"
    rosimage.data = image_np.tostring()
    
    return rosimage, timestamp

def createImuMessge(timestamp_int, omega, alpha):
    timestamp_nsecs = str(timestamp_int)
    timestamp = rospy.Time( int(timestamp_nsecs[0:-9]), int(timestamp_nsecs[-9:]) )
    
    rosimu = Imu()
    rosimu.header.stamp = timestamp
    rosimu.angular_velocity.x = float(omega[0])
    rosimu.angular_velocity.y = float(omega[1])
    rosimu.angular_velocity.z = float(omega[2])
    rosimu.linear_acceleration.x = float(alpha[0])
    rosimu.linear_acceleration.y = float(alpha[1])
    rosimu.linear_acceleration.z = float(alpha[2])
    
    return rosimu, timestamp

def playAVideo(videoFilename):
    print 'videoFilename', videoFilename
    max_height = 500
    cap = cv2.VideoCapture(videoFilename)
    rate = cap.get(cv2.CAP_PROP_FPS);
    print "video frame rate", rate;
    mnStartId = 0
    mnFinishId = 1e6
    cap.set(cv2.CAP_PROP_POS_FRAMES, mnStartId); #start from mnStartId, 0 based index
    if(mnFinishId == -1):
        mnFinishId = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)-1)
    else:
        mnFinishId = min(mnFinishId, int(cap.get(cv2.CAP_PROP_FRAME_COUNT)-1))
    print 'start', mnStartId, 'finish', mnFinishId

    mnCurrentId = mnStartId

    while(cap.isOpened()):
        videoFrameId = cap.get(cv2.CAP_PROP_POS_FRAMES)
        if videoFrameId != mnCurrentId:
            print "Expected frame id", mnCurrentId, "and actual one in video", videoFrameId, "differ."
            print "Likely reached end of video file. Note mnFinishId", mnFinishId
            break        
           
        time_frame= cap.get(cv2.CAP_PROP_POS_MSEC)/1000.0
        print 'currentFrameId', mnCurrentId, ' and video timestamp %.9f' % time_frame    
        ret, frame = cap.read()
        if frame is None:
            print 'Empty frame, break the video stream'
            break  
        
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        h,w = gray.shape[:2]
        if h>max_height:
            gray = cv2.pyrDown(gray,dstsize = (w/2,h/2))

        cv2.imshow('frame',gray)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        mnCurrentId += 1
        if mnCurrentId > mnFinishId:
            print 'Exceeding mnFinishId %d' % mnFinishId + ', break the video stream'              
            break            

    cap.release()
    cv2.destroyAllWindows()


def writeVideoToRosBag(bag, videoFilename, video_time_offset, video_from_to):
    print 'videoFilename', videoFilename
    max_height = 640
    cap = cv2.VideoCapture(videoFilename)
    rate = cap.get(cv2.CAP_PROP_FPS);
    print "video frame rate", rate
            
    mnStartId = 0
    mnFinishId = 1e6
    literalvideofromto = list()
    if mnFinishId == -1:
        mnFinishId = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)-1)
    else:
        mnFinishId = int(min(mnFinishId, cap.get(cv2.CAP_PROP_FRAME_COUNT)-1))
    if video_from_to and rate > 1.0:
        mnStartId = int(max(mnStartId, video_from_to[0]*rate))
        mnFinishId = int(min(mnFinishId, video_from_to[1]*rate))
    literalvideofromto.append(max(float(mnStartId)/rate - 1.0, 0.0))
    literalvideofromto.append(float(mnFinishId)/rate + 1.0)
    print 'video frame index start', mnStartId, 'finish', mnFinishId
    cap.set(cv2.CAP_PROP_POS_FRAMES, mnStartId); #start from mnStartId, 0 based index
    mnCurrentId = mnStartId
    framecount = 0
    while(cap.isOpened()):
        videoFrameId = cap.get(cv2.CAP_PROP_POS_FRAMES)
        if videoFrameId != mnCurrentId:
            print "Expected frame id", mnCurrentId, "and actual one in video", videoFrameId, "differ."
            print "Likely reached end of video file. Note mnFinishId", mnFinishId
            break        
           
        time_frame= cap.get(cv2.CAP_PROP_POS_MSEC)/1000.0
        time_frame_offset = time_frame + video_time_offset
        print 'currentFrameId', mnCurrentId, ' and video timestamp %.9f' % time_frame    
        ret, frame = cap.read()
        if frame is None:
            print 'Empty frame, break the video stream'
            break  
        
        image_np = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        h,w = image_np.shape[:2]
        if h>max_height:
            image_np = cv2.pyrDown(image_np,dstsize = (w/2,h/2))

        cv2.imshow('frame',image_np)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        mnCurrentId += 1
        if mnCurrentId > mnFinishId:
            print 'Exceeding mnFinishId %d' % mnFinishId + ', break the video stream'              
            break            
        
       
        decimal,integer = math.modf(time_frame_offset)
        timestamp = rospy.Time( secs=int(integer), nsecs=int(decimal*1e9) )

        rosimage = Image()
        rosimage.header.stamp = timestamp
        rosimage.height = image_np.shape[0]
        rosimage.width = image_np.shape[1]
        rosimage.step = rosimage.width  #only with mono8! (step = width * byteperpixel * numChannels)
        rosimage.encoding = "mono8"
        rosimage.data = image_np.tostring()

        topic_prefix = 'cam0'
        framecount += 1
        bag.write("/{0}/image_raw".format(topic_prefix), rosimage, timestamp)
    cap.release()
    cv2.destroyAllWindows()
    print 'Saved', framecount, 'image messages'
    return literalvideofromto
   

#create the bag
def main():
    parsed = parseArgs()
    
    bag = rosbag.Bag(parsed.output_bag, 'w')
    if not parsed.video is None:
        print 'video', parsed.video, 'time offset', parsed.video_time_offset
        if parsed.video_from_to:
            print 'video_from_to:', parsed.video_from_to
        videotimerange = writeVideoToRosBag(bag, parsed.video, parsed.video_time_offset, parsed.video_from_to)
    elif not parsed.folder is None:
        #write images
        camfolders = getCamFoldersFromDir(parsed.folder)
        for camfolder in camfolders:
            camdir = parsed.folder + "/{0}".format(camfolder)
            image_files = getImageFilesFromDir(camdir)
            for image_filename in image_files:
                image_msg, timestamp = loadImageToRosMsg(image_filename)
                bag.write("/{0}/image_raw".format(camfolder), image_msg, timestamp)
    else:
        raise Exception('Invalid/Empty video file and image folder')
    buffertime = 5 #sec
    #write imu data
    if parsed.imu is None and parsed.folder is None:
        print "Neither a folder nor any imu file is provided. Rosbag will have only visual data"
    elif parsed.imu is None:
        imufiles = getImuCsvFiles(parsed.folder)
        for imufile in imufiles:
            topic = os.path.splitext(os.path.basename(imufile))[0]
            with open(imufile, 'rb') as csvfile:
                reader = csv.reader(csvfile, delimiter=',')
                headers = next(reader, None)
                for row in reader:
                    imumsg, timestamp = createImuMessge(row[0], row[1:4], row[4:7])
                    bag.write("/{0}".format(topic), imumsg, timestamp)
    else:
        imufile = parsed.imu        
        topic = 'imu0'
        with open(imufile, 'rb') as csvfile:
            reader = csv.reader(csvfile, delimiter=',')
            headers = next(reader, None)
            imucount = 0
            for row in reader:
                imumsg, timestamp = createImuMessge(row[0], row[1:4], row[4:7])
                timestamp_nsecs = str(row[0])
                timestampinsec = float(timestamp_nsecs[0:-9])
                if videotimerange and timestampinsec < parsed.video_time_offset + videotimerange[0] - buffertime \
                  or timestampinsec > parsed.video_time_offset + videotimerange[1] + buffertime:
                    continue
                imucount += 1
                bag.write("/{0}".format(topic), imumsg, timestamp)
            print 'Saved', imucount, 'inertial messages'

    bag.close()
    print 'Saved to bag file', parsed.output_bag

if __name__ == "__main__":
    main()

