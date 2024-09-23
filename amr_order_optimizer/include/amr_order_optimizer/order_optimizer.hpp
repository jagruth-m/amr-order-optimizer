#ifndef ORDER_OPTIMIZER_HPP
#define ORDER_OPTIMIZER_HPP

#include <memory>
#include <filesystem>
#include <vector>
#include <thread>
#include <atomic>
#include <unordered_map>
#include <string>
#include <set>
#include <cmath>
#include <yaml-cpp/yaml.h>
#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include "custom_msg/msg/order.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

struct Part {
    std::string name;
    double cx;
    double cy;
};

struct Product {
    int id;
    std::string name;
    std::vector<Part> parts;
};

class OrderOptimizer : public rclcpp::Node
{
public:
    OrderOptimizer();
    double calculate_distance(double x1, double y1, double x2, double y2);
    double goal_x, goal_y, start_x, start_y;
    std::unordered_map<int, std::vector<Part>> sort_product_parts_by_nearest_neighbor(const std::unordered_map<int, std::vector<Part>> &inputProductPartsMap);
    

private:
    std::atomic<bool> stop_threads;
    std::string directory_; 
     
    std::vector<int> products_id;
    std::vector<Product> products_; 
    std::vector<std::thread> threads;

    std::unordered_map<int, std::vector<Part>> productPartsMap;
    std::unordered_map<int, std::vector<Part>> sortedPartsMap;

    rclcpp::Subscription<custom_msg::msg::Order>::SharedPtr order_subscriber;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr position_subscriber;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_publisher;

    void position_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void order_callback(const custom_msg::msg::Order::SharedPtr msg);
    void parse_order_file(const std::filesystem::path &file_path, int order_id);
    
    void reset();
    void create_marker_array(const std::unordered_map<int, std::vector<Part>> &sortedPartsMap);
};

#endif // ORDER_OPTIMIZER_HPP
