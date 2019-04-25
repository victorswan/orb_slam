# This script is to run all the experiments in one program

import os
import subprocess
import time
import signal

# SeqNameList = ['MH_01_easy', 'MH_03_medium', 'MH_05_difficult', 'V2_02_medium', 'not_exist'];
# SeqNameList = ['V1_03_difficult', 'V2_02_medium', 'V2_03_difficult', 'not_exist'];
SeqNameList = ['MH_01_easy', 'MH_02_easy', 'MH_03_medium', 'MH_04_difficult', 'MH_05_difficult', 'V1_01_easy', \
'V1_02_medium', 'V1_03_difficult', 'V2_01_easy', 'V2_02_medium', 'V2_03_difficult', 'not_exist'];

Result_root = '/mnt/DATA/tmp/EuRoC/TRO_Stereo_Demo/'
# Result_root = '/mnt/DATA/tmp/EuRoC/ORB2_Stereo_Baseline/'
# Result_root = '/mnt/DATA/tmp/EuRoC/Lmk_800/ORB2_active_measErr_withFrameInfo/'

Number_GF_List = [800]; # [400, 600, 800, 1000, 1500, 2000]; # 
# Number_GF_List = [60, 80, 100, 150, 200];

Num_Repeating = 1 # 10 # 20 #  5 # 
SleepTime = 1 # 10 # 25

do_Viz = str('true')
do_Rect = str('true')

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
        cmd_mkdir = 'mkdir -p ' + Experiment_dir
        subprocess.call(cmd_mkdir, shell=True)

        for sn, sname in enumerate(SeqNameList):
            
            print bcolors.ALERT + "====================================================================" + bcolors.ENDC

            SeqName = SeqNameList[sn] # + '_blur_5'
            print bcolors.ALERT + "Round: " + str(iteration + 1) + "; Seq: " + SeqName

            # File_Setting = './Examples/Stereo/EuRoC.yaml'
            # File_Setting = '../../ORB_Data/EuRoC_yaml/EuRoC_stereo_lmk400.yaml'
            # File_Setting = '../../ORB_Data/EuRoC_yaml/EuRoC_stereo_lmk600.yaml'
            File_Setting = '../../ORB_Data/EuRoC_yaml/EuRoC_stereo_lmk800.yaml'
            # File_Setting = '../../ORB_Data/EuRoC_yaml/EuRoC_stereo_lmk1000.yaml'
            # File_Setting = '../../ORB_Data/EuRoC_yaml/EuRoC_stereo_lmk1500.yaml'
            # File_Setting = '../../ORB_Data/EuRoC_yaml/EuRoC_stereo_lmk2000.yaml'
            # File_Setting = '../../ORB_Data/EuRoC_yaml/EuRoC_stereo_lmk2500.yaml'

            # File_Vocab   = '../ORB_Data/ORBvoc.txt'
            File_Vocab   = '../../ORB_Data/ORBvoc.bin'
            File_rosbag  = '/mnt/DATA/Datasets/EuRoC_dataset/BagFiles/' + SeqName + '.bag'
            File_traj = Experiment_dir + '/' + SeqName

            cmd_slam   = str('rosrun GF_ORB_SLAM2 Stereo ' + File_Vocab + ' ' + File_Setting + ' ' + str(int(num_gf*2)) \
                + ' ' + do_Viz + ' ' + do_Rect + ' /cam0/image_raw /cam1/image_raw ' + File_traj)
            cmd_rosbag = 'rosbag play ' + File_rosbag # + ' -r 0.3' # + ' -u 20' 

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

