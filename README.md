Observability estimator
================

Structure
------------------
### **lidar_sub.py**, node name *lidar_observability*, entry name *lidar*
 
Node *lidar_observability* has two subscribers:
1. The primary subscriber **lidar_subscription** for topic *rslidar_points*,
from where it gets the PointCloud2 data. When a message is received, its callback
uses function *make_obs_lidar* from file **lidar2img.py**. Desired parameters 
for *make_obs_lidar* should be set here. The returned images from *make_obs_lidar* are then published,
in the *lidar_observability_matrix*, *lidar_occlusions_matrix* and *lidar_observability_detailed_image*. 
*lidar_observability_matrix* is the topic that is to be used for the AI.
2. And the optional **state_subscription** for topic *vehicle_state*. This subscriber
gets the vehicle position and longitudinal velocity and updates it in the respective
class variables. Variable *viewer_position* is used as an argument for the 
*make_obs_lidar* function, while *longitudinal_velocity* can be used for the AI if
needed. If these functionalities are not needed, this subscriber is to be commented
out, leaving *viewer_position* defaulted to (0,0).

### **lidar2img.py**, primary function *make_obs_lidar*

Parameters are explained in its docstring.

Returns:
1. *Observability matrix*, which is Black(np.uint8(0)) and White(np.uint8(255)).
This matrix is to be used as input for the AI.
2. *Occlusions matrix*, where all the PCL2 points that are higher than the viewer
are shown. Pixel positions represent the same coordinates as in the 
*Observability matrix*. Pixels with value np.uint8(0) represent places with no
PCL2 point higher than the viewer, while the value of the other pixels is the 
same as the height of the latest compiled PCL2 point in that x-y position.
`occlusions = np.where(occlusions != 0, np.uint8(255), occlusions)`
can be used to make all non-zero pixels white.
3. *Inclusive image*, is the enlarged version of the previous two matrices, 
500x500 pixels. It is BGR color coded. The occlusions are highlighted in red color
, while the observability remains white. Additionaly, a small area around the viewer
position is highlited as green. This image serves only for easier visualisation.

### **bb_sub.py**, node name *bounding_box_observability*, entry name *bb*
 
Node *bounding_box_observability* has two subscribers:
1. The primary subscriber **bb_subscription** for topic *trajectory_bbox_array*,
from where it gets the bounding boxes markers. When a message is received, its callback
uses function *make_obs_bb* from file **bb2img.py**. Desired parameters 
for *make_obs_bb* should be set here. The returned images from *make_obs_bb* are then published,
in the *bb_observability_matrix*, *bb_occlusions_matrix* and *bb_observability_detailed_image*. *bb_observability_matrix*
is the topic that is to be used for the AI.
2. And the optional **state_subscription** for topic *vehicle_state*. This subscriber
gets the vehicle position and longitudinal velocity and updates it in the respective
class variables. Variable *viewer_position* is used as an argument for the 
*make_obs_lidar* function, while *longitudinal_velocity* can be used for the AI if
needed. If these functionalities are not needed, this subscriber is to be commented
out, leaving *viewer_position* defaulted to (0,0).

### **bb2img.py**, primary function *make_obs_bb*

Parameters are explained in its docstring. All the bouding boxes are assigned a value of 3, so
higher than the viewer.

Returns:
1. *Observability matrix*, which is Black(np.uint8(0)) and White(np.uint8(255)).
This matrix is to be used as input for the AI.
2. *Occlusions matrix*, where all the bounding boxes in the matrix have a pixel value of their height,
and other parts have a value of np.uint8(0). `occlusions = np.where(occlusions != 0, np.uint8(255), occlusions)`
can be used to make all non-zero pixels white.
3. *Inclusive image*, is the enlarged version of the previous two matrices, 
500x500 pixels. It is BGR color coded. The occlusions are highlighted in red color
, while the observability remains white. Additionaly, a small area around the viewer
position is highlited as green. This image serves only for easier visualisation.

### **image_sub.py**, node name *image_subscriber*, entry name *image*
 
Node *image_subscriber* has six subscribers, one for each image topic that **lidar_observability** and
**bounding_box_observability** nodes can publish. These subscribers' callback function shows the images in a window.
The callback function can be used as an example of turning Image.msg messages into arrays, via the *CvBridge*.
Additionally each subscriber can be commented out as desired if certain topic images do not need to be shown.



How to use, explained through a bag test
------------------
1. Clone or pull git repo to ~/Software/Llagami_BEP (where i cloned it yesterday), as I updated the project for easier accessibility

2. Since this is the first time the package is being built, the `colcon build` command should be used. This 
should also be done when a change is made in one of the package files. Open terminal in the project directory. Here:
- run `colcon build`

3. First make sure that the bag folder is located in the project directory (eg. Project/bag, same as Project/src). 
The required bag folder can be found in ~/Software/Llagami_BEP, where it can be copied from if the git repo is cloned
and not pulled into that directory. Now open the previously opened terminal or a new terminal in the project directory, and run:
- run `. install/setup.bash`
- run `ros2 bag play bag1 -l`

4. To run the **image_subscriber** node, open a new terminal in the project directory. 
Since the packages are already built, run:
- `. install/setup.bash`
- `ros2 run observability image`

5. To run the **lidar_observability** node, open a new terminal in the project directory and run:
- `. install/setup.bash`
- `ros2 run observability lidar` and the image windows should show in a bit.

6. To run the **bounding_box_observability** node, open a new terminal in the project directory and run:
- `. install/setup.bash`
- `ros2 run observability bb` and the image windows should show in a bit.

7. To run the RVIZ2 visualization, open a new terminal in the project directory and run:
- `ros2 run rviz2 rviz2`

8. To show rqt_graph or rqt_console for showing the logs, open a new terminal in the project directory and run:
- `rqt_graph` or `ros2 run rqt_console rqt_console` respectively


Adding the AI
------------------

### Option 1 - AI.main called in callback function
The AI files can be added in the project structure, and be called directly from the **lidar_observability** and
**bounding_box_observability** nodes' Lidar and BB subscriber callback functions, in the designated commented out lines.
In this case, the input for the AI should be *estimations[0]*, which holds the observability matrix. 


The *estimations[0]* is a sensor_msgs.msg.Image message, and needs to be converted to an array before use.
This can be done as shown in the *image_callback* function in the **image_subscriber** node, with a 'mono8'
encoding.


Publisher *ai_result*, can be used to publish the AI result in case it is needed to be type Float64.

### Option 2 - new AI focused node
An AI focused node can be made. The node can have a subscriber for topic, *lidar_observability* and/or *bb_observability*.
The message needs to be turned into an array, as shown in the *image_callback* function in the **image_subscriber** node, with a 'mono8'
encoding.

A new publisher should then be made to publish the AI result.
