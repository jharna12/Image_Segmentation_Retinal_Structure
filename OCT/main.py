# ImageFilter for using filter() function
from PIL import Image, ImageFilter
from numpy import asarray
import numpy
from matplotlib import pyplot as plt
import sys
import glob
import time
import os

print('************Execution of Main File Started*********************')
try:
    print('Removing already existing file from the destination')
    os.remove('final_summary.txt')
except:
    pass

try:
    print('***********Loading OCT_NEW.py Script*************')
    os.system('python oct_new.py')
    print('***********Loading Normal.py Script**************')
    os.system('python normal.py')
except:
    print('Script Files are not present in destination!')
try:
    try:
        # opening the file in read mode
        my_file = open("normal_matrix.txt", "r")
    except:
        print('******Normal_matrix.txt file is not present')
    
    # reading the file
    data = my_file.read()
    
    # replacing end splitting the text
    # when newline ('\n') is seen.
    data_into_list_n = data.split("\n")
    print(data_into_list_n)
    my_file.close()
    
    try:
        my_file = open("defected_matrix.txt", "r")
    except:
        print('********Defected Matrix file is not present.')
    
    # reading the file
    data = my_file.read()
    
    # replacing end splitting the text
    # when newline ('\n') is seen.
    data_into_list_d = data.split("\n")
    print(data_into_list_d)
    my_file.close()
    
    print('*********Execution of comparison of result set started******')
    file = open('final_summary.txt','w+')
    for i in range(0,len(data_into_list_d)-1):
      #  for j in range(0,len(data_into_list_n)-1):
            if(data_into_list_d[i]>data_into_list_n[0] or data_into_list_d[i]>data_into_list_n[1] or data_into_list_d[i]>data_into_list_n[2]):
                content='There is a defect in your eye!!!'
                file.write(content)
                file.write('\n')
                print('There is a defect in your eye!!!')
            else:
                content = 'Volla your eye is normal!!!'
                file.write(content)
                file.write('\n')
                print('Volla your eye is normal!!!')
except:
    print('***********Issue is reading or writing files**********')
print('*************Execution of Main File Completed*************')
            
        