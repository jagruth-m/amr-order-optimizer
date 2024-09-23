# amr-order-optimizer
This repository contains the [ROS](https://www.ros.org/) Package for the given task.

## Setting up
- Clone the repository
- Place the package folder **amr_order_optimizer** inside the **src** of your **workspace**
- Build the package using `colcon build --packages-select amr_order_optimizer`
- After building the package, run the node using launch file and input argument. It directly assigns the path to a directory containing two subdirectories **order** and **configuration**.
`ros2 launch amr_order_optimizer order_optimizer_launch.py directory:=/path/to/directory/containing/folders`
- In the another terminal, publish the order id and description to the topic /nextOrder in the custom message format as shown.
`ros2 topic pub --once /nextOrder custom_msg/msg/Order "{order_id: 1200028, description: 'order1'}"`
