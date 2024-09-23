#include <gtest/gtest.h>
#include "amr_order_optimizer/order_optimizer.hpp" 
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "custom_msg/msg/order.hpp"


TEST(OrderOptimizerTest, TestCalculateDistance)
{
    rclcpp::init(0, nullptr);
    auto optimizer = std::make_shared<OrderOptimizer>();
    double distance = optimizer->calculate_distance(0.0, 0.0, 3.0, 4.0);
    EXPECT_EQ(distance, 5.0); // 3-4-5 triangle, result should be 5
    rclcpp::shutdown();
}


TEST(OrderOptimizerTest, TestNearestNeighborSorting)
{
    rclcpp::init(0, nullptr);
    auto optimizer = std::make_shared<OrderOptimizer>();

    std::unordered_map<int, std::vector<Part>> input_map = {
        {1, {{"PartA", 10.0, 10.0}, {"PartB", 20.0, 20.0}}},
        {2, {{"PartC", 30.0, 30.0}}}
    };
    optimizer->start_x = 0.0;
    optimizer->start_y = 0.0;
    auto sorted_map = optimizer->sort_product_parts_by_nearest_neighbor(input_map);

    ASSERT_EQ(sorted_map.size(), 2);
    EXPECT_EQ(sorted_map[1][0].name, "PartA");
    EXPECT_EQ(sorted_map[1][1].name, "PartB");
    EXPECT_EQ(sorted_map[2][0].name, "PartC");
    rclcpp::shutdown();
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    int ret = RUN_ALL_TESTS();
    return ret;
}
