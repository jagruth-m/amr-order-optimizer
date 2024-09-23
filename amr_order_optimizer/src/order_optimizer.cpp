#include "amr_order_optimizer/order_optimizer.hpp"
#include <iostream>

namespace fs = std::filesystem;
using std::placeholders::_1;

OrderOptimizer::OrderOptimizer() : Node("OrderOptimizer")
{
    // Subscriber for currentPosition topic
    position_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "currentPosition", 10, std::bind(&OrderOptimizer::position_callback, this, _1));
    
    // Subscriber for the nextOrder topic
    order_subscriber = this->create_subscription<custom_msg::msg::Order>(
    "nextOrder", 10, std::bind(&OrderOptimizer::order_callback, this, _1));

    // Publisher for currentPosition topic
    publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("currentPosition", 10);
    
    // Publisher for marker_array
    marker_array_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("order_path", 10);

    this->declare_parameter<std::string>("directory", "/home/jagruth/Documents/amr_example_ROS/applicants_amr_example_1");  //directory path parameter (set via launch file)
    directory_ = this->get_parameter("directory").as_string();

    fs::path configuration_path = fs::path(directory_) / "configuration/products.yaml"; //file path

    try 
    {
        YAML::Node config = YAML::LoadFile(configuration_path.string());

        for (const auto &node : config)
        {
            Product product;
            product.id = node["id"].as<int>();
            product.name = node["product"].as<std::string>();

            for (const auto &partNode : node["parts"]) 
            {
                Part part;
                part.name = partNode["part"].as<std::string>();
                part.cx = partNode["cx"].as<double>();
                part.cy = partNode["cy"].as<double>();
                product.parts.push_back(part);
            }
            products_.push_back(product); // Saves data
        }

    } 
    catch (const YAML::Exception &e) 
    {
        std::cerr << "Error parsing products.yaml file: " << e.what() << std::endl;
    }
}


void OrderOptimizer::position_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    start_x = msg->pose.position.x;
    start_y = msg->pose.position.y;
}


void OrderOptimizer::order_callback(const custom_msg::msg::Order::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Order ID: '%d' and Description:'%s'", msg->order_id, msg->description.c_str());
    
    stop_threads.store(false); //set flag to false for stopping the threads later

    // Multithreading for parsing files in 'orders' directory
    fs::path orders_path = fs::path(directory_) / "orders";
    
    if (fs::exists(orders_path) && fs::is_directory(orders_path))
    {
        for (const auto &entry : fs::directory_iterator(orders_path))
        {
            if (entry.path().extension() == ".yaml")
            {
                threads.emplace_back(&OrderOptimizer::parse_order_file, this, entry.path(), msg->order_id); //Creating threads
            }
        }
    }

    // Join all threads
    for (auto &t : threads)
    {
        if (t.joinable())
            t.join();
    }

    // Iterate over the vector of product IDs to create a map to store parts corresponding to each product ID
    for (int productId : products_id) 
    {
        for (const auto &product : products_) 
        {
            if (product.id == productId) 
            {
                std::cout << "For product found with ID: " << productId << std::endl;
                std::set<std::string> partNameSet; // Set to track unique part names
                std::vector<Part> uniqueParts;

                for (const auto &part : product.parts) 
                {
                    // If part name is not already in the set, add it
                    if (partNameSet.find(part.name) == partNameSet.end())
                    {
                        uniqueParts.push_back(part);
                        partNameSet.insert(part.name);
                    }
                }

                // Store the unique parts for the current product ID
                productPartsMap[productId] = uniqueParts;
            
                for (const auto &part : productPartsMap[productId]) 
                {
                    std::cout << "Part: " << part.name << ", cx: " << part.cx << ", cy: " << part.cy << std::endl;
                }

                break;  // Exit the inner loop once the product is found
            }
        }
    }

    if (productPartsMap.empty()) 
    {
        RCLCPP_INFO(this->get_logger(), "No parts to collect.");
        return;
    }

    //print information
    std::cout << "Working on order " << msg->order_id << " (" << msg->description << ")" << std::endl;
    std::cout << "Starting from: (" << start_x << ", " << start_y << ")" << std::endl;
    sort_product_parts_by_nearest_neighbor(productPartsMap);
    std::cout << "Delivering to destination x: " << goal_x << ", y: " << goal_y << std::endl;
    
    //publishing current position for next orders
    auto current_position = geometry_msgs::msg::PoseStamped();
    current_position.pose.position.x = goal_x;
    current_position.pose.position.y = goal_y;
    publisher->publish(current_position);

    //create Marker Array
    create_marker_array(sortedPartsMap);

    reset();
}


//sorts the part locations for geometrically shortest path
std::unordered_map<int, std::vector<Part>> OrderOptimizer::sort_product_parts_by_nearest_neighbor(
const std::unordered_map<int, std::vector<Part>> &inputProductPartsMap)
{
    std::unordered_map<int, std::vector<bool>> visited_part; //to track visited parts for each product
    
    for (const auto &[productId, parts] : inputProductPartsMap) 
    {
        visited_part[productId] = std::vector<bool>(parts.size(), false); //initialize visited flags
    }

    double current_x = start_x;
    double current_y = start_y;

    while (true) 
    {
        int nearest_product_id = -1;
        int nearest_part_idx = -1;
        double min_distance = std::numeric_limits<double>::max(); //set to the largest value
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

        visited_part[nearest_product_id][nearest_part_idx] = true;
        const auto &nearest_part = inputProductPartsMap.at(nearest_product_id)[nearest_part_idx];

        std::cout << "Fetching part '" << nearest_part.name << "' for product ' " << nearest_product_id << "' at (" << nearest_part.cx << ", " << nearest_part.cy << ")" << std::endl;
        sortedPartsMap[nearest_product_id].push_back(nearest_part);

        current_x = nearest_part.cx;
        current_y = nearest_part.cy;
    }

    return sortedPartsMap;
}

// Function to calculate distance between two points (x1, y1) and (x2, y2)
double OrderOptimizer::calculate_distance(double x1, double y1, double x2, double y2) 
{
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}

//Function to clear the vectors
void OrderOptimizer::reset()
{
    productPartsMap.clear();
    products_id.clear();
    threads.clear();
    sortedPartsMap.clear();
}


// Function to parse an order file (called from a thread)
void OrderOptimizer::parse_order_file(const fs::path &file_path, int order_id)
{
    while (!stop_threads.load())
    {
        YAML::Node order_data = YAML::LoadFile(file_path.string());
        for (const auto &order : order_data)
        {
            int parsed_order_id = order["order"].as<int>();
            if (parsed_order_id == order_id)
            {
                goal_x = order["cx"].as<float>();
                goal_y = order["cy"].as<float>();
                RCLCPP_INFO(this->get_logger(), "For the given Order ID: %d, goal_x: %.2f, goal_y: %.2f", order_id, goal_x, goal_y); //destination and order id
                
                //products for given order
                auto products = order["products"];
                for (const auto &product : products)
                {
                    int product_id = product.as<int>();
                    products_id.push_back(product_id);
                    RCLCPP_INFO(this->get_logger(), "Product ID: %d", product_id);
                }
                // Set the stop flag to true to signal other threads to stop
                stop_threads.store(true);
                break;
            }
        }
    }
}


// Configures and publishes the markers array
void OrderOptimizer::create_marker_array(const std::unordered_map<int, std::vector<Part>> &sortedPartsMap)
{
    visualization_msgs::msg::MarkerArray marker_array;

    // Marker for the AMR's start position
    visualization_msgs::msg::Marker amr_marker;
    amr_marker.header.frame_id = "map";
    amr_marker.header.stamp = this->now();
    amr_marker.ns = "amr_position";
    amr_marker.id = 0;
    amr_marker.type = visualization_msgs::msg::Marker::CUBE;
    amr_marker.action = visualization_msgs::msg::Marker::ADD;
    amr_marker.pose.position.x = start_x/100; 
    amr_marker.pose.position.y = start_y/100;
    amr_marker.pose.position.z = 0.0/100;
    amr_marker.scale.x = 0.5;
    amr_marker.scale.y = 0.5;
    amr_marker.scale.z = 0.5;
    amr_marker.color.r = 0.0f;
    amr_marker.color.g = 1.0f;
    amr_marker.color.b = 0.0f;
    amr_marker.color.a = 1.0f;
    marker_array.markers.push_back(amr_marker);

    // Markers for part pickup locations
    int part_id = 1;
    for (const auto &[productId, sortedParts] : sortedPartsMap)
    {
        for (const auto &part : sortedParts)
        {
            visualization_msgs::msg::Marker part_marker;
            part_marker.header.frame_id = "map";
            part_marker.header.stamp = this->now();
            part_marker.ns = "part_pickup";
            part_marker.id = part_id++;
            part_marker.type = visualization_msgs::msg::Marker::CYLINDER;
            part_marker.action = visualization_msgs::msg::Marker::ADD;
            part_marker.pose.position.x = part.cx/100;
            part_marker.pose.position.y = part.cy/100;
            part_marker.pose.position.z = 0.0/100;
            part_marker.scale.x = 0.2;
            part_marker.scale.y = 0.2;
            part_marker.scale.z = 0.5; 
            part_marker.color.r = 1.0f;
            part_marker.color.g = 0.0f;
            part_marker.color.b = 0.0f;
            part_marker.color.a = 1.0f;
            marker_array.markers.push_back(part_marker);
        }
    }

    // Publish the marker array
    marker_array_publisher->publish(marker_array);
}


//main function
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OrderOptimizer>());
  rclcpp::shutdown();
  return 0;
}