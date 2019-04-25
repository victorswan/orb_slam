# This script is to run all the experiments in one program

import os
import subprocess
import time
import signal


SeqIdxList =  [2, 7, 8, 9, 99]; # [2,99]; #
Result_root = '/mnt/DATA/tmp/KITTI/Mono_noLC/'
Number_GF_List = [1000];
Num_Repeating = 10 # 1 # 
FrameRate = 10 # 5 # 
SleepTime = 1 # 25

#----------------------------------------------------------------------------------------------------------------------
class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    ALERT = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

if FrameRate < 10:
    fR = '0'+str(FrameRate)
else:
    fR = str(FrameRate)

for ri, num_gf in enumerate(Number_GF_List):

    Experiment_prefix = 'LmkNumber_' + str(int(num_gf))

    for iteration in range(0, Num_Repeating):

        Experiment_dir = Result_root + Experiment_prefix + '_Round' + str(iteration + 1)
        cmd_mkdir = 'mkdir ' + Experiment_dir
        subprocess.call(cmd_mkdir, shell=True)

        FirstLoop = True
        for sn, SequenceIdx in enumerate(SeqIdxList):
            
            print bcolors.ALERT + "====================================================================" + bcolors.ENDC

            SeqIdx = str(SeqIdxList[sn]).zfill(2)
            print bcolors.ALERT + "Round: " + str(iteration + 1) + "; Seq: " + SeqIdx + "; fR: " + fR

            File_Setting = './Examples/Monocular/KITTI.yaml'
            File_Vocab   = './Vocabulary/ORBvoc.txt'
            File_rosbag  = '/mnt/DATA/Datasets/Kitti_Dataset/BagFiles/Seq' + SeqIdx + '_' + '10' + '_NonLoopClosure.bag'
            File_traj = Experiment_dir + '/' + SeqIdx

            cmd_slam   = str('rosrun ORB_SLAM2 Mono ' + File_Vocab + ' ' + File_Setting + ' /cam0/image_raw ' + File_traj)
            cmd_rosbag = 'rosbag play ' + File_rosbag # + ' -r 0.3'
            print bcolors.WARNING + "cmd_slam: \n"   + cmd_slam   + bcolors.ENDC
            print bcolors.WARNING + "cmd_rosbag: \n" + cmd_rosbag + bcolors.ENDC

            print bcolors.OKGREEN + "Launching SLAM" + bcolors.ENDC
            proc_slam = subprocess.Popen(cmd_slam, shell=True)

            print bcolors.OKGREEN + "Sleeping for a few secs to wait for voc loading" + bcolors.ENDC
            time.sleep(SleepTime)

            print bcolors.OKGREEN + "Launching rosbag" + bcolors.ENDC
            proc_bag = subprocess.call(cmd_rosbag, shell=True)

            print bcolors.OKGREEN + "Finished rosbag playback, kill the process" + bcolors.ENDC
            subprocess.call('rosnode kill Mono', shell=True)
            time.sleep(SleepTime)
            subprocess.call('pkill Mono', shell=True)