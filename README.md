In this project, we perform perception tasks to discern different objects, and accordingly pick and place them in designated destinations.

Ray Tang 07/18/2018

The pipeline is built as below: 

1. RGB-D camera Image 
2. Voxel Grid Downsampling 
3. Passthrough along Z-Axis
4. Passthrough along X-Axis 
5. Outlier Filter to remove staistical noise 
6. RANSAC PLANE Filter
7. DBSCAN clustering
8. Apply a trained SVM classifier to discern and label objects
9. Generate Yaml files for picking and placing objects

[The python script can be found here.](https://github.com/onearth00/RoboND-Perception-Project/blob/master/pr2_robot/scripts/pipeline.py)

### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

### Exercise 1, 2 and 3 pipeline implemented
#### 1. Complete Exercise 1 steps. Pipeline for filtering and RANSAC plane fitting implemented.

I found that a two-step passthrough filtering (step 3 and 4 in the pipeline) is necessary to clean out things other than the objects of interests.

#### 2. Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented.  

The challenge was to find the right parameters for the clustering (here in particular the cluster tolerance, min, and max cluster size).

#### 2. Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented.

I modified the provided script in a few noticable places, including using HSV instead of RGB, and increasing the number of extractions from 5 to 100 for each object. The training results were satisifing, as shown in the below figure. an averaging 91% accuracy was achieved in the 8-object classification task.

![SVM accuracy](https://github.com/onearth00/RoboND-Perception-Project/blob/master/confusion%20matrix.png)

### Pick and Place Setup

#### 1. For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format.

We achieved 100% accuracy in classifying objects in all three test worlds, and successfully outputted the corresponding yaml files. 

Spend some time at the end to discuss your code, what techniques you used, what worked and why, where the implementation might fail and how you might improve it if you were going to pursue this project further.  



