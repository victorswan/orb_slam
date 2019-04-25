# This script is to run all the experiments in one program

import os
import subprocess
import time
import signal

# SeqNameList = ['MH_01_easy', 'MH_03_medium', 'MH_05_difficult', 'V2_02_medium', 'not_exist'];
# SeqNameList = ['V1_03_difficult', 'V2_02_medium', 'V2_03_difficult', 'not_exist'];
SeqNameList = ['MH_01_easy', 'MH_02_easy', 'MH_03_medium', 'MH_04_difficult', 'MH_05_difficult', 'V1_01_easy', 'V2_01_easy', 'V2_02_medium', 'not_exist'];
Result_root = '/mnt/DATA/tmp/EuRoC/ORB2_Mono_Baseline/'
# Result_root = '/mnt/DATA/tmp/EuRoC/Lmk_800/ORB2_active_measErr_withFrameInfo/'
Number_GF_List = [400, 600, 800, 1000, 1500, 2000]; # [100];
# Number_GF_List = [60, 80, 100, 150, 200];
Num_Repeating = 10 # 20 #  5 # 
SleepTime = 1 # 10 # 25
config_path = '/home/yipuzhao/ros_workspace/package_dir/ORB_Data/'

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

        FirstLoop = True
        for sn, sname in enumerate(SeqNameList):
            
            print bcolors.ALERT + "====================================================================" + bcolors.ENDC

            SeqName = SeqNameList[sn]
            print bcolors.ALERT + "Round: " + str(iteration + 1) + "; Seq: " + SeqName

            # File_Setting = '../ORB_Data/EuRoC_yaml/EuRoC_lmk400.yaml'
            # File_Setting = '../ORB_Data/EuRoC_yaml/EuRoC_lmk600.yaml'
            # File_Setting = '../ORB_Data/EuRoC_yaml/EuRoC_lmk800.yaml'
            # File_Setting = '../ORB_Data/EuRoC_yaml/EuRoC_lmk1000.yaml'
            # File_Setting = '../ORB_Data/EuRoC_yaml/EuRoC_lmk1500.yaml'
            File_Setting = config_path + '/EuRoC_yaml/EuRoC_lmk2000.yaml'
            # File_Setting = '../ORB_Data/EuRoC_yaml/EuRoC_lmk2500.yaml'

            # File_Vocab = config_path + '/ORBvoc.txt'
            File_Vocab = config_path + '/ORBvoc.bin'
            File_rosbag  = '/mnt/DATA/Datasets/EuRoC_dataset/BagFiles/' + SeqName + '.bag'
            File_traj = Experiment_dir + '/' + SeqName

            # rosrun ORB_SLAM2 Mono PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE
            # enable viz
            # cmd_slam   = str('rosrun ORB_SLAM2 Mono ' + File_Vocab + ' ' + File_Setting + ' '  + str(int(num_gf)) + ' true /cam0/image_raw ' + File_traj)
            # disable viz
            cmd_slam   = str('rosrun ORB_SLAM2 Mono ' + File_Vocab + ' ' + File_Setting + ' '  + str(int(num_gf)) + ' false /cam0/image_raw ' + File_traj)
            
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
