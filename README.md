# Particle Filter Project

## Implementation Plan

### Names of Members:

- Diogo Viveiros
- Sam Shatzkin

### Plan

- initialize_particle_cloud: We will initialize our particle cloud by mapping our environment with SLAM initially. Afterwards, we will place an extremely large amount of particles throughout this environment, essentially "filling" this space with randomly distributed particles which we can then use to estimate the robot's position later.
- update_particles_with_motion_model: In order to update the position of the particles, we will use a similar methodology utilized in class when learning the Monte Carlo Localization method. Therefore, we will move these particles in the direction where the particle is facing, and then get the new hypothetical location for that particle. 
- update_particle_weights_with_measurement_model: We will compute the importance weights of each particle by  comparing the robot's sensor readings to the estimated ones from the particle. Each particle will have it's own weight calculated by summing up the difference, taking the absolute value, and then getting the inverse. 
- normalize_particles/ resample_particles: We will normalize these weights by adding up all of the probabilities, and then divide this sum by a number that will give us 1. We will then resample by using a random sampling algorithm that is probabilistically proportion to the weights. 
- update_estimated_robot_pose: Then, we will update the estimated pose of the robot by running a series of tests to determine how cmd_vel instructions relate to change in position of the robot. Additionally, we'll use the robot's odometry data to help us estimate more accurate positions of the robot.
- Noise: We will incorporate noise into our particle filter localization by adding some slight offsets to our final estimations in each step, with the goal of trying to get the particles incrementally closer to the  actual position as possible. 

### Timeline

		- initialize_particle_cloud: Done by April 15th
		- update_particles_with_motion_model: Done by April 20th
		- pdate_particle_weights_with_measurement_model: Done by April 22nd
		- normalize_particles & resample_particles: Done by April 22nd
		- update_estimated_robot_pose: Done by April 24th
		- Noise: Done by April 20th
		- Testing and Refinement: From 24th to 26th
