# This script is to run all the experiments in one program

import os
import subprocess
import time
import signal

SeqNameList = ['2017-12-29-11-45-30'];
# SeqNameList = ['2017-10-30-18-50-57', '2017-10-30-20-04-41', '2017-10-30-18-28-11'];
Result_root = '/mnt/DATA/tmp/Euclid/'
Number_GF_List = [1000];
Num_Repeating = 10 # 20 #  5 # 
SleepTime = 25

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
    
    Experiment_prefix = 'LmkNumber_' + str(int(num_gf))

    for iteration in range(0, Num_Repeating):

        Experiment_dir = Result_root + Experiment_prefix + '_Round' + str(iteration + 1)
        cmd_mkdir = 'mkdir ' + Experiment_dir
        subprocess.call(cmd_mkdir, shell=True)

        FirstLoop = True
        for sn, sname in enumerate(SeqNameList):
            
            print bcolors.ALERT + "====================================================================" + bcolors.ENDC

            SeqName = SeqNameList[sn]
            print bcolors.ALERT + "Round: " + str(iteration + 1) + "; Seq: " + SeqName

            File_Setting = '../ORB_Data/Euclid_yaml/Euclid.yaml'
            File_Vocab   = './Vocabulary/ORBvoc.txt'
            File_rosbag  = '/mnt/DATA/Datasets/TSRB_Euclid/' + SeqName + '.bag'
            File_traj = Experiment_dir + '/' + SeqName

            # rosrun ORB_SLAM2 Mono PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE
            cmd_slam   = str('rosrun ORB_SLAM2 Mono ' + File_Vocab + ' ' + File_Setting + ' /camera/fisheye/image_raw ' + File_traj)
            cmd_rosbag = 'rosbag play ' + File_rosbag + ' -s 10'
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
