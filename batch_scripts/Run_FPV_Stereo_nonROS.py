# This script is to run all the experiments in one program

import os
import subprocess
import time
import signal

SeqNameList = ['indoor_forward_3', 'indoor_forward_5', 'indoor_forward_6', \
               'indoor_forward_7', 'indoor_forward_9', 'indoor_forward_10'];
# SeqNameList = ['indoor_forward_5'];

# Note
# when testing with pre-compute FAST, keep in mind that the detection results 
# of baseline ORB and GF are different: ORB detection is from rectified image,
# while GF detection is from distorted raw image.

Result_root = '/mnt/DATA/tmp/UZH_FPV/GF_GGraph_Stereo_Speedx'
# Result_root = '/mnt/DATA/tmp/UZH_FPV/GF_SWF_Stereo_Speedx'
# Result_root = '/mnt/DATA/tmp/UZH_FPV/GF_Covis_Stereo_Speedx'
# Result_root = '/mnt/DATA/tmp/UZH_FPV/GF_BaseBA_Stereo_Speedx'
# Result_root = '/mnt/DATA/tmp/UZH_FPV/ORB_BaseBA_Stereo_Speedx'
# Result_root = '/mnt/DATA/tmp/UZH_FPV/Debug_Stereo_Speedx'

Playback_Rate_List = [1.0, 2.0, 3.0] 

# Optimal param for ORB
# Number_GF_List = [800]; 
# Number_GF_List = [600, 800, 1000, 1500]; # [200, 300, 400, 600, 800, 1000, 1500, 2000]; # [2000]; # 

# Optimal param for GF
Number_GF_List = [80]; # [130]; 
# Number_GF_List = [80, 100, 130, 160]; # [80, 100, 130, 160, 200, 240, 280, 320, 360]; #  

Num_Repeating = 10 # 1 # 

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

for pi, rate in enumerate(Playback_Rate_List):
    for ri, num_gf in enumerate(Number_GF_List):
        
        Experiment_prefix = 'ObsNumber_' + str(int(num_gf))

        for iteration in range(0, Num_Repeating):

            Experiment_dir = Result_root + str(rate) + '/' \
             + Experiment_prefix + '_Round' + str(iteration + 1)
            cmd_mkdir = 'mkdir -p ' + Experiment_dir
            subprocess.call(cmd_mkdir, shell=True)

            for sn, sname in enumerate(SeqNameList):
                
                subprocess.call('clear all', shell=True)
                print bcolors.ALERT + "====================================================================" + bcolors.ENDC

                SeqName = SeqNameList[sn]
                print bcolors.ALERT + "Round: " + str(iteration + 1) + "; Seq: " + SeqName

                File_Setting = '../../ORB_Data/UZH_FPV_yaml/UZH_FPV_stereo_lmk800.yaml'

                # File_Vocab   = '../../ORB_Data/ORBvoc.txt'
                File_Vocab  = '../../ORB_Data/ORBvoc.bin'

                Path_Image  = '/mnt/DATA/Datasets/UZH_FPV/' + SeqName + '/'
                File_plan   = '../../ORB_Data/UZH_FPV_POSE_GT/' + SeqName + '_cam0.txt'
                File_traj   = Experiment_dir + '/' + SeqName
                File_map    = File_traj + '_map'

                # enable viz
                # cmd_slam   = str('../Examples/Stereo/stereo_general ' \
                #    + '--path_to_vocab ' + File_Vocab + ' ' \
                #    + '--path_to_setting ' + File_Setting + ' ' \
                #    + '--path_to_sequence ' + Path_Image + ' ' \
                #     + '--path_to_planned_track ' + File_plan + ' ' \
                #    + '--path_to_track ' + File_traj + ' ' \
                #    + '--path_to_map ' + File_map + ' ' \
                #    + '--constr_per_frame ' + str(int(num_gf)*2) + ' ' \
                #    + '--budget_per_frame ' + str(int(33/rate + 0.5)) + ' ' \
                #    + '--do_viz');
                # disable viz
                cmd_slam   = str('../Examples/Stereo/stereo_general ' \
                    + '--path_to_vocab ' + File_Vocab + ' ' \
                    + '--path_to_setting ' + File_Setting + ' ' \
                    + '--path_to_sequence ' + Path_Image + ' ' \
                    + '--path_to_planned_track ' + File_plan + ' ' \
                    + '--path_to_track ' + File_traj + ' ' \
                    + '--path_to_map ' + File_map + ' ' \
                    + '--constr_per_frame ' + str(int(num_gf)*2) + ' ' \
                    + '--budget_per_frame ' + str(int(33/rate + 0.5)));

                # make dir for detection I/O
                cmd_mkdir = 'mkdir ' + Path_Image + '/cam0/kpt'
                print bcolors.WARNING + "cmd_mkdir: \n"  + cmd_mkdir  + bcolors.ENDC
                proc_mkdir = subprocess.call(cmd_mkdir, shell=True)

                print bcolors.WARNING + "cmd_slam: \n"   + cmd_slam   + bcolors.ENDC

                print bcolors.OKGREEN + "Launching SLAM" + bcolors.ENDC
                # proc_slam = subprocess.Popen(cmd_slam, shell=True)
                proc_slam = subprocess.call(cmd_slam, shell=True)

                # print bcolors.OKGREEN + "Sleeping for a few secs to wait for voc loading" + bcolors.ENDC
                # time.sleep(SleepTime)

                # print bcolors.OKGREEN + "Finished playback, kill the process" + bcolors.ENDC
                # subprocess.call('pkill Mono', shell=True)