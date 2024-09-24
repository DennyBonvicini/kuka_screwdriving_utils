#include <rclcpp/rclcpp.hpp>
#include <force_sensor/srv/check_touch.hpp>
#include <memory>
#include <control_msgs/msg/interface_value.hpp>
#include <cmath>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <chrono>
#include <thread>
#include <future>
#include <mutex>
#include <condition_variable>


using CheckTouch = force_sensor::srv::CheckTouch;
using InterfaceValue = control_msgs::msg::InterfaceValue;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

class CheckTouchService : public rclcpp::Node
{
public:
    CheckTouchService()
        : Node("check_touch_service"),
          force_x_(0.0), force_y_(0.0), force_z_(0.0), norma_(0.0)
    {
        internal_node_ = std::make_shared<rclcpp::Node>("__internal_node");
        service_ = this->create_service<CheckTouch>(
                    "force_sensor/check_touch",
                    std::bind(&CheckTouchService::handle_service, this, std::placeholders::_1, std::placeholders::_2)
                    );

        // Creazione della nuova sottoscrizione
        subscription_ = internal_node_->create_subscription<InterfaceValue>(
                    "/ft_ati_controller/inputs", 1,
                    std::bind(&CheckTouchService::topic_callback, this, std::placeholders::_1)
                    );

        // Creazione del client dell'azione
        action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(this, "/kuka_scaled_fjt_controller/follow_joint_trajectory");

    }

private:

    void handle_service(const std::shared_ptr<CheckTouch::Request> request,
                        std::shared_ptr<CheckTouch::Response> response)
    {
        soglia_ = request->soglia;
        contact_detected_ = false;
        norma_ = -1;

        RCLCPP_INFO(this->get_logger(), "In attesa del contatto con soglia %.2f", soglia_);

        // Avvia un thread separato per monitorare il contatto
        while (rclcpp::ok() && !contact_detected_) {
            rclcpp::spin_some(internal_node_); // Elabora i callback

            if (norma_ != -1 && norma_ > soglia_) {
                // Cancellazione del movimento
                if (goal_handle_) {
                    auto cancel_result = action_client_->async_cancel_goal(goal_handle_);
                    RCLCPP_INFO(this->get_logger(), "Goal cancellato.");
                }

                contact_detected_ = true;
                break;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }



        // Risposta del servizio
        if (contact_detected_) {
            response->contact = true;
            response->message = "CONTATTO AVVENUTO";
            RCLCPP_INFO(this->get_logger(), "Contatto avvenuto. Norma: %.2f", norma_);
        }
        else
        {
            response->contact = false;
            response->message = "MANCATO CONTATTO";
        }
    }

    void topic_callback(const InterfaceValue::SharedPtr msg)
    {
        force_x_ = 0.0;
        force_y_ = 0.0;
        force_z_ = 0.0;

        // Estrazione dei valori di forza dai messaggi ricevuti
        for (size_t i = 0; i < msg->interface_names.size(); ++i) {
            if (msg->interface_names[i] == "analog_inputs/force_x") {
                force_x_ = msg->values[i];
            } else if (msg->interface_names[i] == "analog_inputs/force_y") {
                force_y_ = msg->values[i];
            } else if (msg->interface_names[i] == "analog_inputs/force_z") {
                force_z_ = msg->values[i];
            }
        }

        //calcolo norma delle forze x,y,z
        norma_= std::sqrt(force_x_ * force_x_ + force_y_ * force_y_ + force_z_ * force_z_);
        RCLCPP_INFO(this->get_logger(), "Norma letta: %.2f ", norma_);
    }

    rclcpp::Node::SharedPtr internal_node_;
    rclcpp::Service<CheckTouch>::SharedPtr service_;
    rclcpp::Subscription<InterfaceValue>::SharedPtr subscription_;
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
    rclcpp_action::ClientGoalHandle<FollowJointTrajectory>::SharedPtr goal_handle_;

    //memorizzo i valori di forza per poi andare a pubblicarli e renderli visibili
    double force_x_;
    double force_y_;
    double force_z_;
    double norma_;
    bool contact_detected_;
    double soglia_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CheckTouchService>());
    rclcpp::shutdown();
    return 0;
}

