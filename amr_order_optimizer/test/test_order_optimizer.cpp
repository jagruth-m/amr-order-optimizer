#include <gtest/gtest.h>
#include "amr_order_optimizer/order_optimizer.hpp"  // Include your OrderOptimizer class header
#include "geometry_msgs/msg/pose_stamped.hpp"  // For PoseStamped message
#include "custom_msg/msg/order.hpp"  // For Order message

std::unordered_map<int, std::vector<Part>> sortedPartsMap;

// Function to calculate Euclidean distance between two points (x1, y1) and (x2, y2)
double calculate_distance(double x1, double y1, double x2, double y2) 
{
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}

std::unordered_map<int, std::vector<Part>> sort_product_parts_by_nearest_neighbor(
const std::unordered_map<int, std::vector<Part>> &inputProductPartsMap)
{
    
    // Create a map to track visited parts for each product
    std::unordered_map<int, std::vector<bool>> visited_part;
    
    for (const auto &[productId, parts] : inputProductPartsMap) 
    {
        visited_part[productId] = std::vector<bool>(parts.size(), false); // Initialize visited flags
    }

    double current_x = 0;
    double current_y = 0;

    while (true) 
    {
        int nearest_product_id = -1;
        int nearest_part_idx = -1;
        double min_distance = std::numeric_limits<double>::max(); // Initially set to the largest value
        bool found_unvisited_part = false;

        // Iterate over each product and its parts to find the nearest unvisited part
        for (const auto &[productId, parts] : inputProductPartsMap) 
        {
            for (size_t i = 0; i < parts.size(); ++i) 
            {
                if (!visited_part[productId][i]) 
                {
                    found_unvisited_part = true;
                    double distance = calculate_distance(current_x, current_y, parts[i].cx, parts[i].cy);
                    if (distance < min_distance) 
                    {
                        min_distance = distance;
                        nearest_product_id = productId;
                        nearest_part_idx = i;
                    }
                }
            }
        }

        // If no unvisited parts were found, break out of the loop
        if (!found_unvisited_part) 
        {
            break;
        }

        // Mark the nearest part as visited
        visited_part[nearest_product_id][nearest_part_idx] = true;

        // Fetch the nearest part
        const auto &nearest_part = inputProductPartsMap.at(nearest_product_id)[nearest_part_idx];
        
        //std::cout << "Fetching part '" << nearest_part.name << "' for product ' " << nearest_product_id << "' at (" << nearest_part.cx << ", " << nearest_part.cy << ")" << std::endl;

        // Store the fetched part in the sorted map
        sortedPartsMap[nearest_product_id].push_back(nearest_part);

        // Update the current position
        current_x = nearest_part.cx;
        current_y = nearest_part.cy;
    }

    return sortedPartsMap;
}

TEST(OrderOptimizerTest, TestCalculateDistance)
{
    //OrderOptimizer optimizer;
    //auto optimizer = std::make_shared<OrderOptimizer>();
    double distance = calculate_distance(0.0, 0.0, 3.0, 4.0);
    EXPECT_EQ(distance, 5.0); // 3-4-5 triangle, result should be 5
}


TEST(OrderOptimizerTest, TestNearestNeighborSorting)
{
    
    // Mock product parts
    std::unordered_map<int, std::vector<Part>> input_map = {
        {1, {{"PartA", 10.0, 10.0}, {"PartB", 20.0, 20.0}}},
        {2, {{"PartC", 30.0, 30.0}}}
    };

    //optimizer.setStartPosition(0.0, 0.0);  // Assuming setter for start position
    auto sorted_map = sort_product_parts_by_nearest_neighbor(input_map);

    ASSERT_EQ(sorted_map.size(), 2);
    EXPECT_EQ(sorted_map[1][0].name, "PartA");
    EXPECT_EQ(sorted_map[1][1].name, "PartB");
    EXPECT_EQ(sorted_map[2][0].name, "PartC");
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);  // Initialize ROS 2 in tests
    int ret = RUN_ALL_TESTS();
    rclcpp::shutdown();  // Shutdown ROS 2 after tests
    return ret;
}
