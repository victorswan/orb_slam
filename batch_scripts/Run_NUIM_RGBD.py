# This script is to run all the experiments in one program

import os
import subprocess
import time
import signal

# SeqNameList = ['office_room_traj0', 'not_exist'];
SeqNameList = ['living_room_traj0', 'living_room_traj1', 'living_room_traj2', 'living_room_traj3', 'office_room_traj0', 'office_room_traj1', 'office_room_traj2', 'office_room_traj3', 'not_exist'];

Result_root = '/mnt/DATA/tmp/NUIM/Lmk_600/ORBv2_info_dispErr/'

Number_GF_List = [60, 80, 100, 130, 160, 200, 240]; # [30, 40, 60, 80, 100, 120, 160, 200]; # [60, 80, 100, 120, 160, 200];  # [100]; # [80, 100, 120]; # 
# Number_GF_List = [400, 600, 800, 1000, 1500, 2000]; # [1000]; # 
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

    Experiment_prefix = 'ObsNumber_' + str(int(num_gf))

    for iteration in range(0, Num_Repeating):

        Experiment_dir = Result_root + Experiment_prefix + '_Round' + str(iteration + 1)
        cmd_mkdir = 'mkdir ' + Experiment_dir
        subprocess.call(cmd_mkdir, shell=True)

        for sn, sname in enumerate(SeqNameList):
            
            print bcolors.ALERT + "====================================================================" + bcolors.ENDC

            SeqName = SeqNameList[sn]
            print bcolors.ALERT + "Round: " + str(iteration + 1) + "; Seq: " + SeqName

            # File_Setting = '../ORB_Data/NUIM_yaml/NUIM_lmk400.yaml'
            File_Setting = '../ORB_Data/NUIM_yaml/NUIM_lmk600.yaml'
            # File_Setting = '../ORB_Data/NUIM_yaml/NUIM_lmk800.yaml'
            # File_Setting = '../ORB_Data/NUIM_yaml/NUIM_lmk1000.yaml'
            # File_Setting = '../ORB_Data/NUIM_yaml/NUIM_lmk1500.yaml'
            # File_Setting = '../ORB_Data/NUIM_yaml/NUIM_lmk2000.yaml'
            
            # File_Vocab   = '../ORB_Data/ORBvoc.txt'
            File_Vocab   = '../ORB_Data/ORBvoc.bin'
            File_rosbag  = '/mnt/DATA/Datasets/ICL-NUIM_dataset/BagFiles/' + SeqName + 'n_rgbd.bag'
            File_traj = Experiment_dir + '/' + SeqName

            # do viz
            # cmd_slam   = str('rosrun GF_ORB_SLAM2 RGBD ' + File_Vocab + ' ' + File_Setting + ' ' + str(int(num_gf*2)) + ' true /camera/image_raw /depth/image_raw ' + File_traj)
            # no viz
            cmd_slam   = str('rosrun GF_ORB_SLAM2 RGBD ' + File_Vocab + ' ' + File_Setting + ' ' + str(int(num_gf*2)) + ' false /camera/image_raw /depth/image_raw ' + File_traj)
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
            subprocess.call('rosnode kill RGBD', shell=True)
            time.sleep(SleepTime)
            subprocess.call('pkill RGBD', shell=True)