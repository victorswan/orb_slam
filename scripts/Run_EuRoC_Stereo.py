#!/usr/bin/env python
# -*-coding:utf-8 -*-
'''
@file Run_EuRoC_Stereo.py
@author Yanwei Du (duyanwei0702@gmail.com)
@date 09-09-2022
@version 1.0
@license Copyright (c) 2022
@desc None
'''


# This script is to run all the experiments in one program

import os
import subprocess
import time

DATA_ROOT = '/mnt/DATA/Datasets/EuRoC'
SeqNameList = [
    'MH_01_easy', 'MH_02_easy', 'MH_03_medium',
    'MH_04_difficult', 'MH_05_difficult',
    'V1_01_easy', 'V1_02_medium', 'V1_03_difficult',
    'V2_01_easy', 'V2_02_medium', 'V2_03_difficult']
RESULT_ROOT = os.path.join(
    os.environ['SLAM_RESULT'], 'gf_orb_slam2/EuRoC/Stereo/')
NumRepeating = 5
SleepTime = 5  # 10 # 25 # second
FeaturePool = [400, 800]  #
SpeedPool = [1.0, 2.0, 3.0, 4.0, 5.0]  # x
EnableViewer = 0
EnableLogging = 1
GF_ORB_SLAM2_PATH = os.path.join(os.environ['IVALAB'], 'gf_orb_slam2/')
CONFIG_PATH = os.path.join(os.environ['IVALAB'], 'good_feature/ORB_Data')

# ----------------------------------------------------------------------------------------------------------------------


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    ALERT = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


for feature in FeaturePool:

    feature_str = str(feature)
    result_1st_dir = os.path.join(RESULT_ROOT, feature_str)

    # loop over play speed
    for speed in SpeedPool:

        speed_str = str(speed)
        result_dir = os.path.join(result_1st_dir, 'Fast' + speed_str)

        # create result dir first level
        cmd_mkdir = 'mkdir -p ' + result_dir
        subprocess.call(cmd_mkdir, shell=True)

        # loop over num of repeating
        for iteration in range(NumRepeating):

            # create result dir second level
            experiment_dir = os.path.join(result_dir, 'Round' + str(iteration + 1))
            cmd_mkdir = 'mkdir -p ' + experiment_dir
            subprocess.call(cmd_mkdir, shell=True)

            # loop over sequence
            for sn, sname in enumerate(SeqNameList):

                print(bcolors.ALERT + "====================================================================" + bcolors.ENDC)

                SeqName = SeqNameList[sn]
                print(bcolors.OKGREEN + f'Seq: {SeqName}; Feature: {feature_str}; Speed: {speed_str}; Round: {str(iteration + 1)};')

                # file_data = os.path.join(DATA_ROOT, SeqName + '.bag')
                # file_timestamp = os.path.join(file_data, 'times.txt')
                file_setting = os.path.join(CONFIG_PATH, 'EuRoC_yaml/EuRoC_stereo_lmk' + feature_str + '.yaml')
                file_vocab = os.path.join(CONFIG_PATH, 'ORBvoc.bin')
                file_traj = os.path.join(experiment_dir, SeqName)
                file_dummy_map = file_traj + '_dummy_map.txt'
                file_log = '> ' + file_traj + '_logging.txt' if EnableLogging else ''
                file_data = os.path.join(DATA_ROOT, SeqName)
                file_timestamp = os.path.join(file_data, 'times.txt')
                file_node = os.path.join(GF_ORB_SLAM2_PATH, 'Examples/Stereo/stereo_euroc')

                # compose cmd
                cmd_slam = \
                    file_node + \
                    ' ' + file_vocab + \
                    ' ' + file_setting + \
                    ' ' + feature_str + \
                    ' ' + '0' + \
                    ' ' + str(EnableViewer) + \
                    ' ' + file_data + \
                    ' ' + file_timestamp + \
                    ' ' + file_traj + \
                    ' ' + file_dummy_map + \
                    ' ' + speed_str + \
                    ' ' + file_log

                print(bcolors.WARNING + "cmd_slam: \n" + cmd_slam + bcolors.ENDC)

                print(bcolors.OKGREEN + "Launching SLAM" + bcolors.ENDC)
                subprocess.call(cmd_slam, shell=True)

                time.sleep(SleepTime)
                print(bcolors.OKGREEN + "Finished" + bcolors.ENDC)
                subprocess.call('pkill stereo_euroc', shell=True)

                time.sleep(SleepTime)
