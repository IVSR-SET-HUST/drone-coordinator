# Instructions for using background subtraction algorithm

* **Requirements**

  Intel® RealSense™ D435
  
  Python3, Matlab
  
  Python libraries: pyrealsense2, numpy, cv2, os, scipy.io

* **Descriptions**
  
  [**getdata.py**]() gets depth and color frame from D435 camera and saves to your data folder
  
  [**read_depth_mat_v2.m**]() loads data, uses background subtraction algorithm and saves image result to your result folder

  [**bounding_box.m**]() bounding box function, using on **read_depth_mat_v2.m** to bound detected object
  
* **Scenario**
 
  Fixed Camera captrue single object moving on a straight line.
  
## **Implementation**

**Step 1: Create data folder and result folder**

Create result folder
  
Create data folder, it contains 4 subfolder: depth_mat, depth_img, rgb_mat, rgb_img

 * depth_mat: stores file mat for each depth frame
 
 * depth_img: stores file jpg for each depth frame
 
 * rgb_mat: stores file mat for each color frame
 
 * rgb_img: stores file jpg for each color frame

**Step 2: Get data**

Change line 7 in **getdata.py** to your data folder location 

Run file **getdata.py**:

&ensp;&ensp;`python3 getdata.py`
    
After, you will get all depth mat, depth jpg, rgb mat, rgb jpg in your data folder.
   
**Step 3: Run background subtraction**
   
Notice: 
   
&ensp;&ensp;*If you use GNU Octave instead of Matlab, you need to install image package.*
   
&ensp;&ensp;*Run this line in Octave command:*
    
&ensp;&ensp;&ensp;&ensp;*`pkg install image-2.10.0.tar.gz`*
    
&ensp;&ensp;*Uncomment line 2 and 3 in **read_depth_mat_v2.m***
   
Change line 4 in **read_depth_mat_v2.m** to your depth_mat data folder, and change line 8 to your result folder.
   
Run **read_depth_mat_v2.m**, moving object will be detected, bounded, and distance (in mm) from camera to object will be displayed in the center of object. And image result is stored in your result folder.
   
## References

<a id="1">[1]</a> 
Hong BH., Park K. (2014), "Moving Human Detection Using Motion Depth in Depth Image Sequences". In: Jeong H., S. Obaidat M., Yen N., Park J. (eds) Advances in Computer Science and its Applications. Lecture Notes in Electrical Engineering, vol 279. Springer, Berlin, Heidelberg. https://doi.org/10.1007/978-3-642-41674-3_83

