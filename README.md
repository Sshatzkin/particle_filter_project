# Particle Filter Project

Diogo Viveiros & Sam Shatzkin

![maze_navigating.gif](https://github.com/Sshatzkin/warmup_project/blob/main/maze_navigating.gif)

## Implementation Plan

- initialize_particle_cloud: We will initialize our particle cloud by mapping our environment with SLAM initially. Afterwards, we will place an extremely large amount of particles throughout this environment, essentially "filling" this space with randomly distributed particles which we can then use to estimate the robot's position later.
- update_particles_with_motion_model: In order to update the position of the particles, we will use a similar methodology utilized in class when learning the Monte Carlo Localization method. Therefore, we will move these particles in the direction where the particle is facing, and then get the new hypothetical location for that particle.
- update_particle_weights_with_measurement_model: We will compute the importance weights of each particle by  comparing the robot's sensor readings to the estimated ones from the particle. Each particle will have it's own weight calculated by summing up the difference, taking the absolute value, and then getting the inverse.
- normalize_particles/ resample_particles: We will normalize these weights by adding up all of the probabilities, and then divide this sum by a number that will give us 1. We will then resample by using a random sampling algorithm that is probabilistically proportion to the weights.
- update_estimated_robot_pose: Then, we will update the estimated pose of the robot by running a series of tests to determine how cmd_vel instructions relate to change in position of the robot. Additionally, we'll use the robot's odometry data to help us estimate more accurate positions of the robot.
- Noise: We will incorporate noise into our particle filter localization by adding some slight offsets to our final estimations in each step, with the goal of trying to get the particles incrementally closer to the  actual position as possible.

## Proposed Timeline

- initialize_particle_cloud: Done by April 15th
- update_particles_with_motion_model: Done by April 20th
- pdate_particle_weights_with_measurement_model: Done by April 22nd
- normalize_particles & resample_particles: Done by April 22nd
- update_estimated_robot_pose: Done by April 24th
- Noise: Done by April 20th
- Testing and Refinement: From 24th to 26th

## Objectives description

 The goal of this project was to implement a particle localization filter which finds and follows the location of the turtlebot on a map of the maze that it currently inhabits.

## High-level description

Our approach starts with 10,000 particles randomly distributed accross the map, each representing a possible position and orientation of the robot. Each time that the robot's movement threshold is reached, we first move each particle corresponding with the movement of the robot, and we use a likelihood field algorithm to compare the measurements of the robot's sensors in 8 directions to the simulated measurements that would be returned if the robot was at each of our 10,000 proposed locations. Each particle is weighted according to the similartiy of its simulated measurements to the actual measurements, and then particles are resampled according to this weight. By repeating these steps, and adding noise in the movement and resampling steps, we lead the particles to converge into a cluster on the robot's actual location.

## Project Components

For each of the main steps of the particle filter localization, please provide the following:
Code location (1-3 sentences): Please describe where in your code you implemented this step of the particle filter localization.
Functions/code description (1-3 sentences per function / portion of code): Describe the structure of your code. For the functions you wrote, describe what each of them does and how they contribute to this step of the particle filter localization.
The main steps are,

### Initialization of particle cloud

  We initialize our particle cloud in the __initialize_particle_cloud__ function provided to us as part of the __ParticleFilter__ class. (Lines 167-188)

  Our particle cloud initialization code generates 10,000 particles at random locations within the bounds of our maze map with random yaw rotations. Each of these particles is assigned the weight "1", and then normalize particles is called to ensure the weights of all particles sum to 1, before publishing them.

### Movement model

### Measurement model

  Our measurement model is update in the __update_particle_weights_with_measurement_model__ function. (Line 370) We update our measurement model using the likelihood field approach, using functions defined in the external likelihood_field.py file provided to us during the in-class likelihood field exercise. Additionally, we duplicated the function __compute_prob_zero_centered_gaussian__ from the class exercise into our particle_filter.py document.

  Our __update_particle_weights_with_measurement_model__ function takes the most recent LIDAR range data from the robot, and uses the likelihood field algorithm to calculate the likelihood that each particle would return this LIDAR data given its position and orientation in the map. This probability is used to assign each particle a weight, which will impact that particle's chances of being resampled.

  More specifically, we projected LIDAR data in 8 directions (at 45 degree increments from each other) for each particle, and retrieved the distance to the nearest obstacle to each of these projected beams using likelihood field functions. The __compute_prob_zero_centered_gaussian__ is paramaterized with these distances and with 0.1, and we use the result of this function to update the new weight that is assigned to each particle.

### Resampling

  Our resampling step happens in two functions, __resample_particles__ (line 246) and __draw_random_sample__ (line 66), which is called by __resample particles__.

  The first step to __resample_particles__ is a call to __draw_random_sample__, a function which collects a weighted random sample of 10,000 particles from the existing particle cloud.

  The next step is to add a small amount of noise to these collected particles, to ensure that we maintain diversity in the positions and orientations of the particles before the next update round.

  Finally we update the particle cloud to be equal to this new array of noisy, resampled particles that have been generated.

### Incorporation of noise

We incorporate noise at two steps in the project, durig our movement update step in __movement_model__(line 38), and during our resampling step in __resample_particles__(line 246).

In our movement model, we used a turn-move-turn approach, and added noise to each of the rotations and forward movements. Often, this noise is lost in the resampling process, because when a single point is sampled many times, the final sample loses a lot of this noise. We found our results to be best when we added a little more noise back after the resampling phase, to ensure that the published particle cloud always contained a cloud of slightly different particles.

### Updating estimated robot pose

The estimated robot pose is updated in function __update_estimated_robot_pose__ (line 351).

In order to update the estimated robot pose, we take the average of the x and y coordinates of every particle in the cloud, as well as the average of the yaw of every particle in the cloud. We assign the estimated robot pose to this average position and orientation.

### Optimization of parameters

## Challenges (1 paragraph)

Describe the challenges you faced and how you overcame them.

## Future work (1 paragraph)

If you had more time, how would you improve your particle filter localization?

## Takeaways (at least 2 bullet points with 2-3 sentences per bullet point)

 What are your key takeaways from this project that would help you/others in future robot programming assignments working in pairs? For each takeaway, provide a few sentences of elaboration.