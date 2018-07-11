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

[image1]: ./misc/rover_image.jpg
[image2]: ./calibration_images/example_grid1.jpg
[image3]: ./calibration_images/example_rock1.jpg 

## [Rubric](https://review.udacity.com/#!/rubrics/916/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

Here's the 1st pass writeup for this project submission.

### Notebook Analysis
#### 1. Run the functions provided in the notebook on test images (first with the test data provided, next on data you have recorded). Add/modify functions to allow for color selection of obstacles and rock samples.

![alt text][image1]

#### 1. Populate the `process_image()` function with the appropriate analysis steps to map pixels identifying navigable terrain, obstacles and rock samples into a worldmap.  Run `process_image()` on your test data using the `moviepy` functions provided to create video output of your result. 

The process_image function is broken down into several components. First, we read the rover camera img and transform the perspecive from the src points to the dst points. Next, using the transformed image we using RGB color thresholds to determine simplistically ground terrain vs. rocks/mountains wich are non-navigable. Then we will convert rover coordinates to world coordinates and update the world map based on the world coordinates.

### Autonomous Navigation and Mapping

#### 1. Fill in the `perception_step()` (at the bottom of the `perception.py` script) and `decision_step()` (in `decision.py`) functions in the autonomous mapping scripts and an explanation is provided in the writeup of how and why these functions were modified as they were.

The perception_step() is updated to contain the functions discussed above to read the camera img, do a perspective transform, color threshold, and then build the coordinates map. We also will update the map to show rocks, navigable terrain, and obstacles to be displayed as the rover explores the world.

The decision_step() code is updated to add the ability to handle a scenario where the rover is stuck, must determine it is stuck, and needs to make alternative movements to get un-stuck. We also add the ability to track towards a rock once a rock has been spotted by the camera, and to add console log outputs stating what is happening wtih the various actions the rover is taking.

#### 2. Launching in autonomous mode your rover can navigate and map autonomously.  Explain your results and how you might improve them in your writeup.  

**Note: running the simulator with different choices of resolution and graphics quality may produce different results, particularly on different machines!  Make a note of your simulator settings (resolution and graphics quality set on launch) and frames per second (FPS output to terminal by `drive_rover.py`) in your writeup when you submit the project so your reviewer can reproduce your results.**

I have tested the Unity simulator environment with a 1024x768 screen resolution and good graphic quality as a point of reference.

The code currently is consistently capable of mapping >80% of the explorable environment with >80% map fidelity and usually within 300 seconds can locate and retrieve 3-5 rock samples. I am continuing to adjust the decision criteria and variables as the rover has a difficult time finding the final rock sample once it has retrieved most of the other samples and cleared much of the explorable map. It frequently revisits areas where there either was no rock sample present or where it has already removed the rock sample.


