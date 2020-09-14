# Instructions for using marker to land UAV

* **Requirements**
  
  Visible-Light Camera
  
  Python3
  
  Python libraries: numpy (1.19.1), opencv (4.4.0), aruco (3.1.2)

* **Descriptions**
  
  [**createArucoMarker.py**](./landing/createArucoMarker.py) define, create type of marker and save markers image to your data folder
  
  [**cameraCalibration.py**](./landing/cameraCalibration.py) get the matrix camera and distortion vector to calculate the translation vector and rotation vector

  [**detectMarker.py**](./landing/checkMarker.py) bounding marker, return the identity of marker, distance from center camera to the centre of marker and orientation from the camera to marker
  
* **Scenario**

  Fixed marker and move camera in any direction toward marker
  
## **Implementation**

**Step 1: Create and save marker image in your forlder**

**Step 2: Get the camera matrix and distortion vector**

The cameraCalibration.py inculdes three functions: calibrate, save_coefficients, load_coefficients

**Step 3: Run detectMarker.py**

The result will show on your screen about distance and orientation from the camera with respect to marker



