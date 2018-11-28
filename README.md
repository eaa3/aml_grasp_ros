# ROS Package for Probabilistic Contact Based Grasp Generation

## Dependencies

  1. Probabilistic Grasp Python Library (see <repo-link>)
  2. For erasp execution on Boris (see <repo-link>)


## Building this package


  1. Clone to your catkin workspace, e.g. if your catkin workspace directory is "my_catkin_ws", then do:
    * `cd my_catkin_ws/src`
    * `git clone <this-repo-url>`
    * `cd ../..`
    * `catkin_make`


## Running this package

  1. There is a single launch file that runs all required nodes for generating and executing grasps with the kinova robot.
    * For running on the simulated robot: `[TODO]`
    * For running on the real robot: `[TODO]`

  * Use keyboard keys to generate grasps, options (execute them in order): 
    * Press `O` to learn contact model from demonstrated grasp.
    * Press `A` to acquire point cloud 
    * Press `U` to learn contact query model on given point cloud
    * Press `Y` to generate grasps

  * The GUI interface has a set of buttons with extra functionality:
    * Learn contact model
    * Acquire cloud
    * Learn contact query
    * Generate Grasps
    * Previous/Next grasp solution
    * Execute grasp: executes selected grasp on robot
    * Restart world: restarts gazebo world
    * Clear solutions: clears generated solutions