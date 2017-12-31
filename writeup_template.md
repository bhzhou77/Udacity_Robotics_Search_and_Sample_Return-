## Project: Search and Sample Return
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**The goals / steps of this project are the following:**  

**Training / Calibration**  

* Download the simulator and take data in "Training Mode"
* Test out the functions in the Jupyter Notebook provided
* Add functions to detect obstacles and samples of interest (golden rocks)
* Fill in the `process_image()` function with the appropriate image processing steps (perspective transform, color threshold etc.) to get from raw images to a map.  The `output_image` you create in this step should demonstrate that your mapping pipeline works.
* Use `moviepy` to process the images in your saved dataset with the `process_image()` function.  Include the video you produce as part of your submission.

**Autonomous Navigation / Mapping**

* Fill in the `perception_step()` function within the `perception.py` script with the appropriate image processing functions to create a map and update `Rover()` data (similar to what you did with `process_image()` in the notebook). 
* Fill in the `decision_step()` function within the `decision.py` script with conditional statements that take into consideration the outputs of the `perception_step()` in deciding how to issue throttle, brake and steering commands. 
* Iterate on your perception and decision function until your rover does a reasonable (need to define metric) job of navigating and mapping.  

[//]: # (Image References)

[image1]: rock_sample_filtered.jpg
[image2]: navigable_terrain.jpg
[image3]: rock_sample.jpg
[image4]: example_grid1.jpg
[image5]: warped.jpg

## [Rubric](https://review.udacity.com/#!/rubrics/916/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

### simulator settings: FPS: varies from 15 to 32 if looking at outputs on the terminal during autonomous mode. For the resolution, the image I obtained from the training mode is 320×160. 

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Notebook Analysis
#### 1. Run the functions provided in the notebook on test images (first with the test data provided, next on data you have recorded). Add/modify functions to allow for color selection of obstacles and rock samples.

I used the RGB values to identify navigable terrain, obstacles and rock samples. For navigable terrain, I used the criteria: > 160 in R channel, > 160 in G channel, and > 160 in B channel. For the rock samples, I used the criteria: > 100 in R channel, > 100 in G channel, and < 50 in B channel. 

This figure is the result of the filtered rock sample in the original image before perspective transformation.

![][image1]

This figure is the result of the filtered navigable terrain (white) after perspective transformation.

![][image2]

This figure is the result of the filtered rock sample (white) after perspective transformation.

![][image3]

#### 1. Populate the `process_image()` function with the appropriate analysis steps to map pixels identifying navigable terrain, obstacles and rock samples into a worldmap.  Run `process_image()` on your test data using the `moviepy` functions provided to create video output of your result. 

For any input image, I applied perspective transformation first using the following examaple grid image where I obtain the coordinates of the four corners of the grid by hand.

![][image4]

After the perspective transformation, I obtained the following warped image:

![][image5]

After this, I applied color threshold as discussed above to identify navigable terrain, rock samples and obstacles. Then, I used the function rover_coords to tranform the three groups of pixels into a coordinate centered around the rover. Then, these rover-centric pixels can be mapped to the worldmap using function pix_to_world. The worldmap is a 200 by 200 by 3 numpy array. I used worldmap[:,:,0] to represent obstacles, worldmap[:,:,1] to represent rocks, and worldmap[:,:,2] to represent navigable terrain. The result is shown in the included video with name 'test_mapping.mp4'.

### Autonomous Navigation and Mapping

#### 1. Fill in the `perception_step()` (at the bottom of the `perception.py` script) and `decision_step()` (in `decision.py`) functions in the autonomous mapping scripts and an explanation is provided in the writeup of how and why these functions were modified as they were.

**For the perception part:**

I added two new functions: the first one is border_thresh that is to change the border part (out of the view of rover camera) of the perespective-transformed image to black, and the second one is find_rocks to locate the rock samples.

In the function perception_step:

1) As default.

2) As default.

3) obtained the filtered navigable terrain, filtered border and filtered rock samples.

4) Update the vision_image. All the three channels of the border are set to 0 so that the image that is out of the view of the rover camera will be shown as black.

5) As default.

6) When there is rock sample detected, update the rock position on the worldmap.

7) Update worldmap only when certain criteria is satisfied: roll and pitch angles have to be around 0 or 360 degrees, and the rover must not being picking up. These criteria increase the fidelity of the mapping.

8) Besides the default polar transformation, I also added two new variables. The first one is called nav_weights, which is inversely related to how many times a pixel has been mapped. I think this will at least force the rover to visit new terrains. The second one is called return_weights, which is inversely related to how far one pixel is from the starting pixel. This is used when all the 6 samples have been collected, and will force the rover to return to the original staring point.

**For the decision part:**

1) First, the rover should check whether it is stuck or not. For this, I used a variable called vel_history that record last 160 velocity inputs, and if the mean of these 160 velocities is smaller than 0.2m/s, then the rover is in stuck state, and it should do some turning and backward running to get rid of the stuck state. I added this feature, because I found that the rover sometimes will get stuck with those small obstacles.

2) Second, the rover should check whether all the six samples has been collected or not. If all the six samples have been collected, then it should return to the original starting point using the variable return_weights to calculate its navigation direction. If some samples are still missing, then the rover should continue to search for them.

3) Third, if the rover detects some rock sample, it should approach that sample steadily.

4) Last, if no sample is detected, the rover should explore the terrain.

5) I also added some Gaussian noise to the navigation direction, just to make it a little bit more random.


#### 2. Launching in autonomous mode your rover can navigate and map autonomously.  Explain your results and how you might improve them in your writeup.  

Given time, I think my rover can always pick up the 6 rock samples and send them back. [This link](https://www.youtube.com/watch?v=bhfFAU6IHso&t=687s) is for one of the example videos. Sometimes, my rover can finish really quickly using less than 15 mins. But sometimes, it can use more than 30 mins. In the above example video, the rover used only 4 mins to picked up the first 5 samples, but more than 20 mins to obtain the last one! There are two reasons: First, when the rover is in a relatively open area, the rover tend to loop in a circle, and usually it takes a long time for the rover to get out. Second, the rover will be stuck by the small obstacles and also takes some time to get out. The fist one is partly solved by the added noise on the navigation angle, but I think a more elegant solution is needed, such as keeping a memory of the terrain just visited. The second one is partly solved by telling the rover to do some turning and backward running. But this does not always work, since sometimes even I cannot make the rover out of the stuck state mannually. This may need a better hardware design or a smarter navigation algorithms trying to avoid those small obstacles in advance.

Other problems are, for example, the rover has detected the rock sample, but it just passes that sample without stopping. This is due to the high speed of the rover, and lowering the maximum velocity may help. However a better solution could be that the rover keep a record of the located rocks and picked rocks, and return to the locations of those located-but-not-yet-picked ones. In addtion, a better perception algorithm may also help.

**Note: running the simulator with different choices of resolution and graphics quality may produce different results, particularly on different machines!  Make a note of your simulator settings (resolution and graphics quality set on launch) and frames per second (FPS output to terminal by `drive_rover.py`) in your writeup when you submit the project so your reviewer can reproduce your results.**






