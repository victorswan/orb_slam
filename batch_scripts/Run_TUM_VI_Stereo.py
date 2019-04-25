# This script is to run all the experiments in one program

import os
import subprocess
import time
import signal


SeqNameList = ['room1_512_16', 'not_exist'];
# SeqNameList = ['room1_512_16', 'room2_512_16', 'room3_512_16', 'room4_512_16', 'room5_512_16', 'room6_512_16', 'not_exist'];

Result_root = '/mnt/DATA/tmp/TUM_VI/ORBv2_Stereo_Baseline/'

Number_GF_List = [1500]; # [30, 60, 80, 100, 120, 160]; 
# Number_GF_List = [300, 400, 600, 800, 1000, 1500, 2000];
Num_Repeating = 10 # 3 # 1 #  
SleepTime = 2 # 10 # 25

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

for ri, num_gf in enumerate(Number_GF_List):

    Experiment_prefix = 'ObsNumber_' + str(int(num_gf))

    for iteration in range(0, Num_Repeating):

        Experiment_dir = Result_root + Experiment_prefix + '_Round' + str(iteration + 1)
        cmd_mkdir = 'mkdir ' + Experiment_dir
        subprocess.call(cmd_mkdir, shell=True)

        for sn, sname in enumerate(SeqNameList):
            
            print bcolors.ALERT + "====================================================================" + bcolors.ENDC

            SeqName = SeqNameList[sn]
            print bcolors.ALERT + "Round: " + str(iteration + 1) + "; Seq: " + SeqName

            File_Setting = './Examples/Stereo/TUM_VI.yaml'
            # File_Setting = './Examples/Stereo/TUM_VI_fov.yaml'
            
            # File_Vocab   = './Vocabulary/ORBvoc.txt'
            File_Vocab   = './Vocabulary/ORBvoc.bin'
            File_rosbag  = '/mnt/DATA/Datasets/TUM_VI/BagFiles/dataset-' + SeqName + '_small_chunks.bag'
            File_traj = Experiment_dir + '/' + SeqName

            cmd_slam   = str('rosrun ORB_SLAM2 Stereo ' + File_Vocab + ' ' + File_Setting + ' ' + str(int(num_gf)) + ' false false /cam0/image_raw /cam1/image_raw ' + File_traj)
            cmd_rosbag = 'rosbag play ' + File_rosbag # + ' -u 20' # + ' -r 0.5'
            
            print bcolors.WARNING + "cmd_slam: \n"   + cmd_slam   + bcolors.ENDC
            print bcolors.WARNING + "cmd_rosbag: \n" + cmd_rosbag + bcolors.ENDC

            print bcolors.OKGREEN + "Launching SLAM" + bcolors.ENDC
            proc_slam = subprocess.Popen(cmd_slam, shell=True)

            print bcolors.OKGREEN + "Sleeping for a few secs to wait for voc loading" + bcolors.ENDC
            time.sleep(SleepTime)

            print bcolors.OKGREEN + "Launching rosbag" + bcolors.ENDC
            proc_bag = subprocess.call(cmd_rosbag, shell=True)

            print bcolors.OKGREEN + "Finished rosbag playback, kill the process" + bcolors.ENDC
            subprocess.call('rosnode kill Stereo', shell=True)
            time.sleep(SleepTime)
            subprocess.call('pkill Stereo', shell=True)
