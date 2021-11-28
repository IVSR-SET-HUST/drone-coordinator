#!/usr/bin/env python



#This code closely follows the pytalker/pylistener node example (http://wiki.ros.org/ROSNodeTutorialPython)
#(c) Yannick Huber (huberya)


import roslib
roslib.load_manifest('msf_updates')
import rospy
import numpy as np
import csv
import os
from math import sqrt
from transformations import euler_from_quaternion, quaternion_from_euler

#import sys
#sys.path.insert(0, '~/catkin_ws/src/ethzasl_msf/msf_core
#need pose_with_covariance (as input for msf) and position if want to apply noise there as well
from geometry_msgs.msg import PointStamped as positiontype
from geometry_msgs.msg import TransformStamped as transformtype
from geometry_msgs.msg import Vector3Stamped as eventtype

"""

$ rosmsg show geometry_msgs/PointStamped 
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
geometry_msgs/Point point
  float64 x
  float64 y
  float64 z
  
$ rosmsg show geometry_msgs/TransformStamped 
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
string child_frame_id
geometry_msgs/Transform transform
  geometry_msgs/Vector3 translation
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion rotation
    float64 x
    float64 y
    float64 z
    float64 w
"""

class MsfNoiseHandler:
  def __init__(self):
    #params for noise. To be set in yaml file
    #if mean is 0 then adding pure noise
    #if mean != 0 then is creating some drift aswell (not yet)
    
    #params for pose
    self.pose_mu_=rospy.get_param("~pose_noise_mean",0.0)
    self.pose_stddeviation_p_=rospy.get_param("~pose_noise_stddeviation_p", 1.0)
    self.pose_stddeviation_q_=rospy.get_param("~pose_noise_stddeviation_q", 1.0)
    self.pose_use_noise_=rospy.get_param("~pose_use_noise", False)
        
    self.pose_p_outlier_=rospy.get_param("~pose_probability_outlier", 0.0)
    self.pose_create_outlier_=rospy.get_param("~pose_create_outlier", False)
    self.pose_group_size_=rospy.get_param("~pose_group_size", 1) #not working rn
    self.pose_curr_group_=0 
    self.pose_noise_type_=rospy.get_param("pose_noise_type", "normal")
    self.pose_curr_frame_=0
    
    #params for training set creation (for noise estimation LSTM)
    self.pose_create_ts_=rospy.get_param("~pose_create_ts", False)
    self.pose_path_to_ts_=rospy.get_param("~pose_path_to_ts", "")
    #this is the sequence length one intends to use for the lstm. choosing it ~ as the frequency works fine
    self.pose_lstm_sequence_length_ =rospy.get_param("~pose_lstm_sequence_length", 10)
    self.pose_original_noise_p_=rospy.get_param("~pose_original_noise_p", 0.0)#this should be the noise that is originally in the data
    self.pose_original_noise_q_=rospy.get_param("~pose_original_noise_q", 0.0)#this should be the noise that is originally in the data
    if self.pose_use_noise_ and self.pose_create_ts_:
        with open(self.pose_path_to_ts_, "a") as myfile:
          myfile.write("starting new training sequence\n")
          
    #these params are for setting meta noise (i.e. non constant noise during a run especially for training)
    self.pose_use_meta_noise_=rospy.get_param("~pose_use_meta_noise", False)
    self.pose_noise_change_frequency_ = round(self.pose_lstm_sequence_length_/2) #play with this
    self.pose_meta_noise_mean_p_ = rospy.get_param("~pose_meta_noise_mean_p", 0.0)
    self.pose_meta_noise_dev_p_ = rospy.get_param("~pose_meta_noise_dev_p", 0.0)
    self.pose_meta_noise_mean_q_ = rospy.get_param("~pose_meta_noise_mean_q", 0.0)
    self.pose_meta_noise_dev_q_ = rospy.get_param("~pose_meta_noise_dev_q", 0.0)
    
    
    #params for position
    self.position_mu_=rospy.get_param("~position_noise_mean",0.0)
    self.position_stddeviation_=rospy.get_param("~position_noise_stddeviation_p", 1.0)
    self.position_use_noise_=rospy.get_param("~position_use_noise", False)
        
    self.position_p_outlier_=rospy.get_param("~position_probability_outlier", 0.0)
    self.position_create_outlier_=rospy.get_param("~position_create_outlier", False)
    self.position_group_size_=rospy.get_param("~position_group_size", 1) #not working rn
    self.position_curr_group_=0
    
	#params for training set creation (for noise estimation LSTM)
    self.position_create_ts_=rospy.get_param("~position_create_ts", False)
    self.position_path_to_ts_=rospy.get_param("~position_path_to_ts", "")
    #this is the sequence length one intends to use for the lstm. choosing it ~ as the frequency works fine
    self.position_lstm_sequence_length_ =rospy.get_param("~position_lstm_sequence_length", 10)
    self.position_original_noise_p_=rospy.get_param("~position_original_noise_p", 0.0)
    if self.position_use_noise_ and self.position_create_ts_:
        with open(self.position_path_to_ts_, "a") as myfile:
          myfile.write("starting new training sequence\n")
          
    #these params are for setting meta noise (i.e. non constant noise during a run especially for training)
    self.position_use_meta_noise_=rospy.get_param("~position_use_meta_noise", False)
    self.position_noise_change_frequency_ = round(self.position_lstm_sequence_length_/2) #play with this
    self.position_meta_noise_mean_p_ = rospy.get_param("~position_meta_noise_mean_p", 0.0)
    self.position_meta_noise_dev_p_ = rospy.get_param("~position_meta_noise_dev_p", 0.0)
    self.position_meta_noise_mean_q_ = rospy.get_param("~position_meta_noise_mean_q", 0.0)
    self.position_meta_noise_dev_q_ = rospy.get_param("~position_meta_noise_dev_q", 0.0)
    
    
    #params for transform (we handle NavSatFix as transform for params)
    self.transform_mu_=rospy.get_param("~transform_noise_mean",0.0)
    self.transform_stddeviation_=rospy.get_param("~transform_noise_stddeviation_p", 1.0)
    self.transform_use_noise_=rospy.get_param("~transform_use_noise", False)
        
    self.transform_p_outlier_=rospy.get_param("~transform_probability_outlier", 0.0)
    self.transform_create_outlier_=rospy.get_param("~transform_create_outlier", False)
    self.transform_group_size_=rospy.get_param("~transform_group_size", 1) #not working rn
    self.transform_curr_group_=0
    self.transform_use_fixed_diverge_time_=rospy.get_param("~transform_diverge_fixed_time", False)
    self.transform_diverge_frame_=rospy.get_param("~transform_diverge_frame", 0)
    self.transform_diverge_length_=rospy.get_param("~transform_diverge_length", 0)
    self.transform_curr_frame_=0
    
	#params for training set creation (for noise estimation LSTM)
    self.transform_create_ts_=rospy.get_param("~transform_create_ts", False)
    self.transform_path_to_ts_=rospy.get_param("~transform_path_to_ts", "")
    self.transform_original_noise_p_=rospy.get_param("~transform_original_noise_p", 0.0)
    if self.transform_use_noise_ and self.transform_create_ts_:
        with open(self.transform_path_to_ts_, "a") as myfile:
          myfile.write("starting new training sequence\n")
    #for transform meta noise use position meta noise for now
    
    #params to estimate stddeviation of data 
    #to be set manually
    #self.ninit_=350
    #this is to allow stable init
    self.ninit_=2000
    self.started_=False
    self.nrecv_=0
    #self.stddeviation_=0
    #self.datamean_=0
    
    #init publisher
    self.position_pub_=rospy.Publisher("noise_drift_handler/position_output", positiontype, queue_size=20)
  #adds gaussian distributed noise with mu and stddeviation(for 3DOF)
  def add_noise3dof(self, arrin, mu, stddeviation):
    noise=np.random.normal(mu, stddeviation, 3)
    arrout=arrin+noise
    return arrout
  #adds gaussian distributed noise with mu and stddeviation-p and stddeviation_q (for 6DOF, supposed to be translation, orientation)
  def add_noise6dof(self, arrin, mu, stddeviation_p, stddeviation_q):
    arrout=arrin
    noise_p=np.random.normal(mu, stddeviation_p, 3)
    arrout[:3]+=noise_p
    #convert quaternion to euler angles and apply noise in that space (more meaningful, not sure if this
    #correct way of doing it though (has different arangement w, x, y, z instead of x, y, z, w
    [ai, aj, ak] = euler_from_quaternion([arrout[6],arrout[3], arrout[4], arrout[5]])
    noise_q=np.random.normal(mu, stddeviation_q, 3)
    ai+=noise_q[0]
    aj+=noise_q[1]
    ak+=noise_q[2]
    #convert back
    q_temp = quaternion_from_euler(ai, aj, ak)
    arrout[3]=q_temp[1]
    arrout[4]=q_temp[2]
    arrout[5]=q_temp[3]
    arrout[6]=q_temp[0]
    return arrout

  #creates an oultier based on arring in by adding or subtracting 100 times stddeviation to arrin
  def create_outlier(self, arrin, stddeviation):
    #add some big disturbance to arrin (for now 100 times stddeviation)
    #sign=np.random.randint(0,2) #need 0/1 random
    #noise=np.array([100*stddeviation for i in range(len(arrin))])
    noise = np.random.normal(0, 100*(stddeviation+0.1), len(arrin))
    #if sign:
    #  arrout=arrin+noise
    #else:
    #  arrout=arrin-noise
    arrout=arrin+noise
    return arrout

  #callback for messages. First decides which of the supported data types (currently pose_with_covariance_stamped
  #and PointStamped) is current input and then manipulates them according to params before outputting them in their
  #corresponding chanel
  def callback(self, data):
    #decide which datatype is current and set params accordingly
    dtype=str(data._type)
    if dtype=="geometry_msgs/TransformStamped":
      dataarr=np.array([data.transform.translation.x, data.transform.translation.y, data.transform.translation.z])
      self.transform_curr_frame_+=1
      #this part is for changing noise and used for creating trainingsets
      if self.position_use_meta_noise_ and self.transform_curr_frame_%self.position_noise_change_frequency_==0:
        self.transform_stddeviation_+=np.random.normal(self.position_meta_noise_mean_p_, self.position_meta_noise_dev_p_)
        #make sure these stay positive
        self.transform_stddeviation_=max(0.0, self.transform_stddeviation_)
      if self.transform_use_fixed_diverge_time_ and self.transform_curr_frame_==self.transform_diverge_frame_:
        self.transform_curr_group_=self.transform_diverge_length_
        eventout = eventtype()
        eventout.header = data.header
        eventout.vector.x = 3
        eventout.vector.y = 0
        eventout.vector.z = 0
        self.events_pub_.publish(eventout)
        print("diverging gps")
      stddeviation=self.transform_stddeviation_
      use_noise=self.transform_use_noise_
      mu=self.transform_mu_
      if use_noise and self.transform_create_ts_:
        with open(self.transform_path_to_ts_, "a") as myfile:
          myfile.write(str(stddeviation+self.position_original_noise_p_)+"\n")
          
      p_outlier=self.transform_p_outlier_
      create_outlier=self.transform_create_outlier_
      group_size=self.transform_group_size_
      curr_group=self.transform_curr_group_
    else:
      print("not supported datatype:")
      print(dtype)

    if(self.started_):
      #change data according to params choosen above
      if use_noise:
        if(len(dataarr)==3):
          dataarr=self.add_noise3dof(dataarr, mu, stddeviation)
        elif(len(dataarr)==7):
          dataarr=self.add_noise6dof(dataarr, mu, stddeviation_p, stddeviation_q)
        else:
          print("unrecognised array length:"+str(len(dataarr)))
      if create_outlier:
        t=np.random.uniform()
        if curr_group>0:
          #print("creating grouped outlier")
          dataarr=self.create_outlier(dataarr, stddeviation)
          if dtype=="geometry_msgs/TransformStamped":
            self.transform_curr_group_-=1
            if self.transform_curr_group_==0:
              eventout = eventtype()
              eventout.header = data.header
              eventout.vector.x = 4
              eventout.vector.y = 0
              eventout.vector.z = 0
              self.events_pub_.publish(eventout)
        elif t<p_outlier:
          dataarr=self.create_outlier(dataarr, stddeviation)
          if dtype=="geometry_msgs/TransformStamped":
            self.transform_curr_group=self.transform_group_size_-1
    else:
      self.nrecv_+=1
      if self.nrecv_>=self.ninit_:
        print("starting noise")
        self.started_=True 
        
    #print(dataarr[0])

 
    if dtype=="geometry_msgs/TransformStamped":
      datan=positiontype()
      datan.header = data.header
      datan.point.x = dataarr[0]
      datan.point.y = dataarr[1]
      datan.point.z = dataarr[2]
      self.position_pub_.publish(datan)
    else:
      print("not supported datatype (output):")
      print(dtype)
    
  def listener(self):
    topic_transform="noise_drift_handler/transform_input"
    rospy.Subscriber(topic_transform, transformtype, self.callback)
    rospy.spin()
      
  def reconfigure(self, config, level):
    return config
      
if __name__ == '__main__':
  try:
    rospy.init_node('noise_drift_handler', anonymous=True)
    gs=MsfNoiseHandler()
    gs.listener()
  except rospy.ROSInterruptException:
    pass
