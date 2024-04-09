#!/usr/bin/env python3
import csv
import os
import matplotlib.pyplot as plt

#DIRECTORY = 'records/'
DIRECTORY = 'records/'
FILE = 'dcmotor_2023-12-30-16-25-30.csv' 




def main(args=None):
    # load data
    desktop = os.path.join(os.path.join(os.path.expanduser('~')), 'Scrivania/turms_ws/') 
    file_name = desktop+DIRECTORY+FILE
    print(os.path)
    time = []
    left_vel = []
    rigth_vel = []
    with open(file_name, 'r', newline='') as file: 
        reader = csv.reader(file, delimiter=' ')
        for row in reader:
            token = row[0].split(",")
            try:
                time.append(int(token[0]))
                left_vel.append(float(token[2]))
                rigth_vel.append(float(token[3]))
            except ValueError:
                None
            print(row)
        file.close()
    
    # graph 
    plt.figure()
    plt.plot(time,left_vel,'-b', label='left')
    plt.plot(time,rigth_vel,'-cdr', label='right')
    plt.ylabel('Vel [rpm]')
    plt.xlabel('Time [ms]')
    plt.legend()
    plt.show()

    return


if __name__ == '__main__':
    main()