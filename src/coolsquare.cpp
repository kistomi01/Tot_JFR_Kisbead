#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>

using namespace std::chrono_literals;  
class SquareTurtle : public rclcpp::Node {
public:
    SquareTurtle() : Node("square_turtle") {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  
            std::bind(&SquareTurtle::move_step, this));
        
        side_length = 2.0;
        turn_angle = M_PI/2;
    }

private:
    void move_step() {
        auto twist = geometry_msgs::msg::Twist();
        
        if (current_step < 4) {
            if (distance_moved < side_length) {
                twist.linear.x = 1.0;
                distance_moved += 0.1;
            } 
            else if (angle_turned < turn_angle) {
                twist.angular.z = 1.0;
                angle_turned += 0.1;
            } 
            else {
                current_step++;
                distance_moved = 0;
                angle_turned = 0;
            }
        } 
        else {
            twist.linear.x = 0;
            twist.angular.z = 0;
            rclcpp::shutdown();
        }
        
        publisher_->publish(twist);
    }

    int current_step = 0;
    double distance_moved = 0;
    double angle_turned = 0;
    double side_length;
    double turn_angle;
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SquareTurtle>());
    rclcpp::shutdown();
    return 0;
}