#!/usr/bin/env python3

import argparse
import subprocess
import csv
import os.path
from send2trash import send2trash
import numpy as np

sequences = [
    "room1",
    "room2",
    "room3",
    "room4",
    "room5",
    "room6"
]


def convert_gt(new_gt_file_name, sequence_path):
    """
    Convert the timestamps of a ground truth sequence from ns to s.

    """

    if os.path.isfile(new_gt_file_name):
        return

    csv_row_dicts = []

    with open("%s/dso/gt_imu.csv" % sequence_path) as old_gt_file:
        old_gt_reader = csv.DictReader(old_gt_file, delimiter=',')
        for row in old_gt_reader:

            csv_row_dicts.append({
                "#timestamp [s]": float(row['# timestamp[ns]']) / 1e9,
                "p_RS_R_x [m]": row['tx'],
                "p_RS_R_y [m]": row['ty'],
                "p_RS_R_z [m]": row['tz'],
                "q_RS_x []": row['qx'],
                "q_RS_y []": row['qy'],
                "q_RS_z []": row['qz'],
                "q_RS_w []": row['qw']
            })

    with open(new_gt_file_name, 'w') as new_gt_file:
        fieldnames = ["#timestamp [s]", "p_RS_R_x [m]", "p_RS_R_y [m]",
                      "p_RS_R_z [m]", "q_RS_x []", "q_RS_y []", "q_RS_z []", "q_RS_w []"]
        new_gt_writer = csv.DictWriter(new_gt_file, fieldnames=fieldnames)

        new_gt_writer.writeheader()
        for row_dict in csv_row_dicts:
            new_gt_writer.writerow(row_dict)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='''
    This script runs over many sequences (specified in 'sequences') of the TUM-VI dataset. Every sequence is executed 'runs_per_sequence' times. The evaluation script from TUM-RGBD (also used by ORB-SLAM3) is used to calculate the Root Mean Square Absolute Trajectory Error (RMS ATE). The median of all runs is reported in 'rmsate_summary.txt'.
    ''')
    parser.add_argument(
        "--dataset_path", help="Path to the TUM-VI dataset. Should lead to a directory that contains the folders '1024_16' and '512_16'.")
    parser.add_argument(
        "--resolution", help="Either '1024_16' or '512_16'. Default: '512_16'", default="512_16")
    parser.add_argument('--runs_per_sequence',
                        help='How often should every sequence be evaluated. Default: 3', default=3)
    args = parser.parse_args()

    dir_path = os.path.dirname(os.path.realpath(__file__))

    if os.path.isfile("%s/rmsate_summary.txt" % dir_path):
        print("An old version of 'rmsate_summary.txt' exists. Going to delete it.")
        send2trash("%s/rmsate_summary.txt" % dir_path)

    with open("rmsate_summary.txt", "a") as summary_file:
        summary_file.write(
            "#sequence name: median RMS ATE, fail count/ runs per sequence\n")

    # run over all sequences
    for sequence in sequences:

        sequence_path = "%s/%s/dataset-%s_%s" % (
            args.dataset_path, args.resolution, sequence, args.resolution)

        print("Looking for a sequence in %s" % sequence_path)

        # initialize statistics
        rmsates = np.zeros(args.runs_per_sequence, dtype=np.float64)
        fail_count = 0

        # execute this sequence runs_per_sequence times
        for run_number in range(args.runs_per_sequence):
            print("Running DSO on sequence %s run number %d" %
                  (sequence, run_number + 1))

            failed = False

            gt_file_name = "%s/gt_%s.txt" % (dir_path, sequence)
            convert_gt(gt_file_name, sequence_path)

            # the result.txt file is the indicator if a run was successful
            # we delete it know to see if a new file exists after DSO finished
            if os.path.isfile("%s/result.txt" % dir_path):
                os.remove("%s/result.txt" % dir_path)

            # execute DSO
            subprocess.run(["%s/../build/bin/dso_dataset" % dir_path,
                            "files=%s/dso/cam0/images" % sequence_path,
                            "calib=%s/dso/cam0/camera.txt" % sequence_path,
                            "gamma=%s/dso/cam0/pcalib.txt" % sequence_path,
                            "vignette=%s/dso/cam0/vignette.png" % sequence_path,
                            "preset=0",
                            "mode=0",
                            "quiet=1",
                            "nolog=1",
                            "nogui=1"],
                           cwd=dir_path)

            # indicator if the run was successful
            if not os.path.isfile("result.txt"):
                failed = True
                fail_count += 1
                print("DSM on sequence %s run number %d FAILED" %
                      (sequence, run_number + 1))

            if not failed:

                print("Calculating RMS ATE for %s run number %d" %
                      (sequence, run_number + 1))

                # Calculate RMS ATE by using the evaluation script from TUM-RGBD (also used by ORB-SLAM3)
                evaluate_ate_scale_proc = subprocess.Popen(["python2", "-u",
                                                            "%s/evaluate_ate_scale.py" % dir_path,
                                                            gt_file_name,
                                                            "%s/result.txt" % dir_path,
                                                            "--max_difference", "0.1",
                                                            "--plot", "plot-%s.svg" % sequence],
                                                           cwd=dir_path,
                                                           universal_newlines=True, stdout=subprocess.PIPE,
                                                           stderr=subprocess.STDOUT)

                stdout = evaluate_ate_scale_proc.communicate()[0]
                # parse the output of the evaluation script
                try:
                    rmsates[run_number] = float(stdout.rstrip().split(',')[2])
                    print("RMSATE: %f" % rmsates[run_number])
                except:
                    print(stdout)

        # get median of runs
        median = np.NaN
        if fail_count < args.runs_per_sequence:
            median = np.median(rmsates[rmsates != 0])

        # write statistics
        with open("%s/rmsate_summary.txt" % dir_path, "a") as summary_file:
            summary_file.write("%s: %f, %d/%d\n" % (sequence,
                                                    median, fail_count, args.runs_per_sequence))

        print("median RMS ATE of %s: %f" % (sequence, median))
        print("failed %d/%d" % (fail_count, args.runs_per_sequence))
