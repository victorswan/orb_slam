# This script is to run all the experiments in one program

import os
import subprocess
import time
import signal


# SeqNameList = ['left_cam', 'right_cam', 'not_exist'];
SeqNameList = ['stereo_cam'];

play_speed = 0.5 # 0.1 # 

Result_root = '/mnt/DATA/tmp/NewCollege/ORBv2_Baseline/'

# Number_GF_List = [60, 80, 100, 130, 160, 200, 240]; # [80, 100, 120]; # 
Number_GF_List = [1500] # [400, 600, 800, 1000, 1500, 2000]; #  
Num_Repeating = 1 # 10 # 50 # 20 #  5 # 
SleepTime = 1 # 10 # 25

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

            File_Setting = '../../ORB_Data/NewCollege_yaml/Bumblebee_stereo_lmk1500.yaml'

            File_Log_details = Experiment_dir + '/' + SeqName + '_details.log'
            
            File_Vocab = '../../ORB_Data/ORBvoc.bin'
            Path_Image  = '/media/yipuzhao/651A6DA035A51611/New_College/Stereo_Images/'
            File_traj = Experiment_dir + '/' + SeqName
            
            # enable viz
            cmd_slam   = str('../Examples/Stereo/stereo_newcollege ' + File_Vocab + ' ' + File_Setting + ' ' + str(int(num_gf)*2) + ' true ' + str(play_speed) + ' ' + Path_Image + ' ' + File_traj)
            # disable viz
            cmd_slam   = str('../Examples/Stereo/stereo_newcollege ' + File_Vocab + ' ' + File_Setting + ' ' + str(int(num_gf)*2) + ' false ' + str(play_speed) + ' ' + Path_Image + ' ' + File_traj)
            
            print bcolors.WARNING + "cmd_slam: \n"   + cmd_slam   + bcolors.ENDC

            print bcolors.OKGREEN + "Launching SLAM" + bcolors.ENDC
            # proc_slam = subprocess.Popen(cmd_slam, shell=True)
            proc_slam = subprocess.call(cmd_slam, shell=True)
