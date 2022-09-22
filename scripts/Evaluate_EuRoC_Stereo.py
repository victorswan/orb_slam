#!/usr/bin/env python
# -*-coding:utf-8 -*-
'''
@file Evaluate_EuRoC_Stereo.py
@author Yanwei Du (duyanwei0702@gmail.com)
@date 09-19-2022
@version 1.0
@license Copyright (c) 2022
@desc None
'''


import glob
import numpy as np
import os
import subprocess
import time


# This script is to run all the experiments in one program


DATA_ROOT = '/mnt/DATA/Datasets/EuRoC/'
SeqNameList = [
    'MH_01_easy', 'MH_02_easy', 'MH_03_medium',
    'MH_04_difficult', 'MH_05_difficult',
    'V1_01_easy', 'V1_02_medium', 'V1_03_difficult',
    'V2_01_easy', 'V2_02_medium', 'V2_03_difficult']
RESULT_ROOT = os.path.join(
    os.environ['SLAM_RESULT'], 'gf_orb_slam2/EuRoC/StereoROSUbuntu1604/')
NumRepeating = 5
SleepTime = 1  # 10 # 25 # second
FeaturePool = [400, 800]
SpeedPool = [1.0, 2.0, 3.0, 4.0, 5.0]  # x
GT_ROOT = os.path.join(DATA_ROOT, 'gt_pose')
SENSOR = 'cam0'
SaveResult = 1
ResultFile = [
    'CameraTrajectory_tracking'
]

# ----------------------------------------------------------------------------------------------------------------------


def call_evaluation(eval, gt, est, options, save):
    cmd_eval = eval + ' ' + gt + ' ' + est + ' ' + options
    if save:
        result = os.path.splitext(est)[0] + '.zip'
        cmd_eval = cmd_eval + ' --save_results ' + result
        if os.path.exists(result):
            cmd_rm_result = 'rm ' + result
            subprocess.call(cmd_rm_result, shell=True)

    print(bcolors.WARNING + "cmd_eval: \n" + cmd_eval + bcolors.ENDC)
    print(bcolors.HEADER + os.path.basename(est))
    subprocess.call(cmd_eval, shell=True)


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

        # loop over num of repeating
        for iteration in range(NumRepeating):

            experiment_dir = os.path.join(result_dir, 'Round' + str(iteration + 1))

            # loop over sequence
            for sn, sname in enumerate(SeqNameList):

                print(bcolors.ALERT + "====================================================================" + bcolors.ENDC)

                SeqName = SeqNameList[sn]
                print(bcolors.OKGREEN + f'Seq: {SeqName}; Feature: {feature_str}; Speed: {speed_str}; Round: {str(iteration + 1)};')

                # create evaluation command
                file_eval = 'evo_ape tum'
                options = '-va --align_origin'

                # gt file
                file_gt = os.path.join(GT_ROOT, SeqName + '_' + SENSOR + '.txt')
                if not os.path.exists(file_gt):
                    print(f'missing gt file: {file_gt}')
                    exit(-1)

                # select result file with the maximum frames
                file_prefix = os.path.join(experiment_dir, SeqName+'_' + ResultFile[0] + '*.txt')
                file_est = ''
                max_frame_num = -1
                for result_file in glob.glob(file_prefix):
                    result_poses = np.loadtxt(result_file)
                    if max_frame_num < result_poses.shape[0]:
                        max_frame_num = result_poses.shape[0]
                        file_est = result_file
                print(f'{file_est}, {max_frame_num}')
                if not os.path.exists(file_est):
                    print(f'missing est file {file_est}')
                    continue
                # evaluate
                call_evaluation(file_eval, file_gt, file_est, options, SaveResult)

                print(bcolors.OKGREEN + "Finished" + bcolors.ENDC)
                time.sleep(SleepTime)
