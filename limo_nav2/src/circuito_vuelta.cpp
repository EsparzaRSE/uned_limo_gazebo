#include "../include/circuito_vuelta.hpp"

namespace uned_limo_gazebo {

const std::string config_dir{ament_index_cpp::get_package_share_directory("uned_limo_gazebo") + "/limo_nav2/config"};

CircuitoVuelta::CircuitoVuelta(const rclcpp::NodeOptions &options) : Node("circuito_vuelta", options){
    this->declare_parameter("location_file", config_dir + "/circuito_lap_locations.yaml");
    RCLCPP_INFO(get_logger(), "Init done");
    
    // Retrasa la ejecución de setup() hasta después de que se haya completado la construcción del componente
    auto timer_callback = [this]() -> void {
        this->setup();
        setup_timer_->cancel();
    };
    setup_timer_ = this->create_wall_timer(std::chrono::milliseconds(1), timer_callback);
}

void CircuitoVuelta::setup(){
    create_behavior_tree();
    const auto timer_period = std::chrono::milliseconds(500);
    timer_ = this->create_wall_timer(timer_period, std::bind(&CircuitoVuelta::update_behavior_tree, this));
}

void CircuitoVuelta::create_behavior_tree(){
    BT::BehaviorTreeFactory factory{};

    BT::NodeBuilder builder = [=](const std::string &name, const BT::NodeConfiguration &config){
        return std::make_unique<GoToPose>(name, config, shared_from_this());
    };

    factory.registerBuilder<GoToPose>("GoToPose", builder);
    tree_ = factory.createTreeFromFile(config_dir + "/circuito_lap_bt_tree.xml");
}

void CircuitoVuelta::update_behavior_tree(){
    BT::NodeStatus tree_status = tree_.tickRoot();

    if(tree_status == BT::NodeStatus::RUNNING){
        return;
    }
    else if(tree_status == BT::NodeStatus::SUCCESS){
        RCLCPP_INFO(this->get_logger(), "Finished Navigation");
    }
    else if(tree_status == BT::NodeStatus::FAILURE){
        RCLCPP_INFO(this->get_logger(), "Navigation failed");
        timer_->cancel();
    }
}

} // namespace uned_limo_gazebo

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(uned_limo_gazebo::CircuitoVuelta)
