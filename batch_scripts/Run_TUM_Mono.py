# This script is to run all the experiments in one program

import os
import subprocess
import time
import signal


SeqNameList = ['freiburg2_pioneer_slam3', 'not_exist'];
# SeqNameList = ['freiburg1_floor', 'freiburg2_pioneer_slam', 'freiburg2_pioneer_slam3', 'freiburg3_long_office_household', 'freiburg3_walking_halfsphere', 'freiburg1_desk', 'freiburg1_desk2', 'freiburg1_room', 'freiburg2_desk', 'freiburg2_xyz', 'freiburg2_desk_with_person', 'not_exist'];
# SeqNameList = ['freiburg1_desk2']
# SettingList = ['TUM2', 'TUM2', 'TUM2', 'TUM2', ' ']; 
Result_root = '/mnt/DATA/tmp/TUM/Mono_noLC/'
Number_GF_List = [1000];
Num_Repeating = 10 # 3 # 5 # 
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

for ri, num_gf in enumerate(Number_GF_List):

    Experiment_prefix = 'LmkNumber_' + str(int(num_gf))
    # Experiment_prefix = 'Refer'

    for iteration in range(0, Num_Repeating):

        Experiment_dir = Result_root + Experiment_prefix + '_Round' + str(iteration + 1)
        cmd_mkdir = 'mkdir ' + Experiment_dir
        subprocess.call(cmd_mkdir, shell=True)

        FirstLoop = True
        for sn, sname in enumerate(SeqNameList):
            
            print bcolors.ALERT + "====================================================================" + bcolors.ENDC

            SeqName = SeqNameList[sn]
            print bcolors.ALERT + "Round: " + str(iteration + 1) + "; Seq: " + SeqName

            File_Setting = './Examples/Monocular/TUM.yaml'
            File_Vocab   = './Vocabulary/ORBvoc.txt'
            File_rosbag  = '/mnt/DATA/Datasets/TUM_rgbd_slam/' + SeqName + '.bag'
            File_traj = Experiment_dir + '/' + SeqName

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