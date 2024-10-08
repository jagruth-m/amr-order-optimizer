# amr-order-optimizer
This repository contains the [ROS](https://www.ros.org/) packages for the given task made using the current LTS Versions 22.04 and [Humble](https://docs.ros.org/en/humble/index.html).

## Setting up
- Clone the repository and place the package folders **amr_order_optimizer** and **custom_msg** inside the **src** of your **workspace**.
- Build the packages inside the workspace directory using `colcon build --packages-select amr_order_optimizer custom_msg`.
- Source the development environment after building the packages `source install/setup.bash`
- After sourcing, run the node using launch file and input argument. The command below launches the node and directly assigns the path to a directory containing two subdirectories **orders** and **configuration**.
```
ros2 launch amr_order_optimizer order_optimizer_launch.py directory:=/path/to/directory/containing/folders
```
- In the another terminal, publish order id and description to the topic */nextOrder* in the custom message format as shown.
```
ros2 topic pub --once /nextOrder custom_msg/msg/Order "{order_id: 1200028, description: 'order1'}"
```
- Example output for the geometrically shortest path: 
```
[OrderOptimizer-1] Working on order 1200028 (order1)
[OrderOptimizer-1] Starting from: (0, 0)
[OrderOptimizer-1] Fetching part 'Part C' for product ' 599' at (281.394, 68.3963)
[OrderOptimizer-1] Fetching part 'Part C' for product ' 691' at (281.394, 68.3963)
[OrderOptimizer-1] Fetching part 'Part C' for product ' 354' at (281.394, 68.3963)
[OrderOptimizer-1] Fetching part 'Part B' for product ' 253' at (550.099, 655.423)
[OrderOptimizer-1] Fetching part 'Part B' for product ' 354' at (550.099, 655.423)
[OrderOptimizer-1] Fetching part 'Part A' for product ' 691' at (791.863, 732.232)
[OrderOptimizer-1] Fetching part 'Part A' for product ' 354' at (791.863, 732.232)
[OrderOptimizer-1] Delivering to destination x: 935.518, y: 469.63
```

- Now, in case of second order, the robot starts from previous destination and reaches the current goal through intermediary points determining the goemetrically shortest path. Example input command:
```
ros2 topic pub --once /nextOrder custom_msg/msg/Order "{order_id: 1400002, description: 'order2'}"
```
Output:
```
[OrderOptimizer-1] Working on order 1400002 (order2)
[OrderOptimizer-1] Starting from: (935.518, 469.63)
[OrderOptimizer-1] Fetching part 'Part A' for product ' 266' at (791.863, 732.232)
[OrderOptimizer-1] Fetching part 'Part A' for product ' 825' at (791.863, 732.232)
[OrderOptimizer-1] Fetching part 'Part A' for product ' 282' at (791.863, 732.232)
[OrderOptimizer-1] Fetching part 'Part A' for product ' 867' at (791.863, 732.232)
[OrderOptimizer-1] Fetching part 'Part A' for product ' 620' at (791.863, 732.232)
[OrderOptimizer-1] Fetching part 'Part B' for product ' 266' at (550.099, 655.423)
[OrderOptimizer-1] Fetching part 'Part B' for product ' 825' at (550.099, 655.423)
[OrderOptimizer-1] Fetching part 'Part B' for product ' 867' at (550.099, 655.423)
[OrderOptimizer-1] Fetching part 'Part C' for product ' 266' at (281.394, 68.3963)
[OrderOptimizer-1] Fetching part 'Part C' for product ' 825' at (281.394, 68.3963)
[OrderOptimizer-1] Fetching part 'Part C' for product ' 282' at (281.394, 68.3963)
[OrderOptimizer-1] Fetching part 'Part C' for product ' 867' at (281.394, 68.3963)
[OrderOptimizer-1] Delivering to destination x: 202.387, y: 35.1363
```

## RViz
- The node publishes a marker array of the message type *visualization_msgs/MarkerArray* on the topic */order_path* where the starting AMR position is indicated with a marker of type ‘CUBE’ while each part pickup location should be visualized with a marker of type ‘CYLINDER’.

- Open **RViz**, click on Add to add **MarkerArray** by topic */order_path*.
- Now, publish the custom message with order id and description in to the topic */nextOrder*
- The node publishes the marker array *visualization_msgs/MarkerArray* with AMR position and pickup location of the parts, and these locations can be visualized in RViz.

## Testing
- [Googletest](https://github.com/google/googletest) is used for testing.
- Before perfomring the tests, replace the default path to directory contains folders 'orders' and 'configuration' line:23 in *src/order_optimizer.cpp* and give the `this->declare_parameter<std::string>("directory", "/path/to/directory/containing/folders");` 
- Two tests with testsuite named **OrderOptimizerTest** are encoded in the **test** folder. Tests can be run using `colcon test --packages-select amr_order_optimizer --event-handlers console_cohesion+`.

