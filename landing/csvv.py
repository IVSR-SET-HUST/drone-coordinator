import pandas as pd
import numpy as np
import csv
import os


#csv_file = os.path.join(os.getcwd()+ './position_gl.csv')
IMG_SIZE = 608
def write_csv(position,size_tag,time_array, csv_file, x_true, y_true, z_true, xx_cam, yy_cam, zz_cam, xx_dr, yy_dr, zz_dr ):
    num_operation = len(position)

    #with open(csv_file, newline='') as myFile:
    x = []
    y = []
    z = []
    time = []
    z_true_array = [z_true]*num_operation
    size_tag_array = [size_tag]*num_operation
    x_true_array = [x_true]*num_operation
    y_true_array = [y_true]*num_operation
    xx = [xx_cam]*num_operation
    yy = [yy_cam]*num_operation
    zz = [zz_cam]*num_operation
    xd = [xx_dr] * num_operation
    yd = [yy_dr] * num_operation
    zd = [zz_dr] * num_operation

      # width2.append(IMG_SIZE)
      # height2.append(IMG_SIZE)
      # filename2.append(filename[i+1])
      # classes2.append(classes[i+1])
      # xmin2.append((int((float(xmin[i+1])*scale_x))))
      # ymin2.append((int((float(ymin[i+1])*scale_y))))
      # xmax2.append((int((float(xmax[i+1])*scale_x))))
      # ymax2.append((int((float(ymax[i+1])*scale_y))))
    for i in range(len(position)):
        x.append(float(position[i, 0]))
        y.append(float(position[i, 1]))
        z.append(float(position[i, 2]))
        time.append(time_array[i])
    value = {'size_tag':size_tag_array,'time':time ,
             'x_true':x_true_array, 'y_true':y_true_array, 'z_true':z_true_array,
             'tx_drone':xd, 'ty_drone':yd, 'tz_drone':zd,
             'tx_cam':xx, 'ty_cam':yy, 'tz_cam':zz,'x':x, 'y':y, 'z':z,
             }
    #print(xmin2[1],ymin2[1],xmax2[1],ymax2[1])
    xml_df = pd.DataFrame(value)
    xml_df.to_csv(csv_file, index= None)
    #print('Successfully converted xml to csv.')


#poo = np.random.rand(10, 3)
#time = np.random.rand(10)
#write_csv(poo,time, 'test.cvs')
