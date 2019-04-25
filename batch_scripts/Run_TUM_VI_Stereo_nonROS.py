# This script is to run all the experiments in one program

import os
import subprocess
import time
import signal


# SeqNameList = ['room1_512_16', 'not_exist'];
SeqNameList = ['room1_512_16', 'room2_512_16', 'room3_512_16', 'room4_512_16', 'room5_512_16', 'room6_512_16', 'not_exist'];

# Result_root = '/mnt/DATA/tmp/TUM_VI/cuda_ORBv2_Stereo_Baseline/'
Result_root = '/mnt/DATA/tmp/TUM_VI/Lmk_600/ORBv2_Stereo_info_dispErr_v10/'
# Result_root = '/mnt/DATA/tmp/TUM_VI/Lmk_600/ORBv2_Stereo_longlive_v10/'

Number_GF_List = [60, 80, 100, 130, 160, 200, 240, 280, 320, 360, 400, 440]; # [200]; # 
# Number_GF_List = [200, 300, 400, 600, 800, 1000, 1500, 2000]; # [1000, 1500]; # 
Num_Repeating = 10 # 1 # 
# SleepTime = 2 # 10 # 25

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

            SeqName = SeqNameList[sn]
            print bcolors.ALERT + "Round: " + str(iteration + 1) + "; Seq: " + SeqName

            # File_Setting = './Examples/Stereo/TUM_VI_fov.yaml'
            # File_Setting = '../../ORB_Data/TUM_VI_yaml/TUM_VI_stereo_lmk300.yaml'
            # File_Setting = '../../ORB_Data/TUM_VI_yaml/TUM_VI_stereo_lmk400.yaml'
            File_Setting = '../../ORB_Data/TUM_VI_yaml/TUM_VI_stereo_lmk600.yaml'
            # File_Setting = '../../ORB_Data/TUM_VI_yaml/TUM_VI_stereo_lmk800.yaml'
            # File_Setting = '../../ORB_Data/TUM_VI_yaml/TUM_VI_stereo_lmk1000.yaml'
            # File_Setting = '../../ORB_Data/TUM_VI_yaml/TUM_VI_stereo_lmk1500.yaml'
            # File_Setting = '../../ORB_Data/TUM_VI_yaml/TUM_VI_stereo_lmk2000.yaml'

            File_Vocab   = '../../ORB_Data/ORBvoc.bin'

            Path_Image   = '/mnt/DATA/Datasets/TUM_VI/dataset-' + SeqName + '/mav0/'
            File_traj = Experiment_dir + '/' + SeqName

            # enable viz
            # cmd_slam   = str('../Examples/Stereo/stereo_general ' + File_Vocab + ' ' + File_Setting + ' ' + str(int(num_gf)*2) + ' true ' + Path_Image + ' ' + File_traj)
            # disable viz
            cmd_slam   = str('../Examples/Stereo/stereo_general ' + File_Vocab + ' ' + File_Setting + ' ' + str(int(num_gf)*2) + ' false ' + Path_Image + ' ' + File_traj)
            
            print bcolors.WARNING + "cmd_slam: \n"   + cmd_slam   + bcolors.ENDC

            print bcolors.OKGREEN + "Launching SLAM" + bcolors.ENDC
            # proc_slam = subprocess.Popen(cmd_slam, shell=True)
            proc_slam = subprocess.call(cmd_slam, shell=True)

            # print bcolors.OKGREEN + "Sleeping for a few secs to wait for voc loading" + bcolors.ENDC
            # time.sleep(SleepTime)

            # print bcolors.OKGREEN + "Finished playback, kill the process" + bcolors.ENDC
            # subprocess.call('pkill Mono', shell=True)

