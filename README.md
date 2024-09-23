# amr-order-optimizer
This repository contains the [ROS](https://www.ros.org/) Package for the given task.

## Setting up
- Clone the repository
- Place the package folder **amr_order_optimizer** inside the **src** of your **workspace**
- Build the package inside the workspace directory using `colcon build --packages-select amr_order_optimizer`
- Source the development environment after building the package `source install/setup.bash`
- After building the package, run the node using launch file and input argument. It directly assigns the path to a directory containing two subdirectories **order** and **configuration**.
```
ros2 launch amr_order_optimizer order_optimizer_launch.py directory:=/path/to/directory/containing/folders
```
- In the another terminal, publish the order id and description to the topic /nextOrder in the custom message format as shown.
```
ros2 topic pub --once /nextOrder custom_msg/msg/Order "{order_id: 1200028, description: 'order1'}"
```
- Example output: 
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
