#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/srv/compute_center.hpp>  // Cambia con il tuo servizio
#include <std_msgs/msg/float64.hpp>

using ComputeCenter = tf::srv::ComputeCenter;

class ComputeCenterService : public rclcpp::Node
{
public:
    ComputeCenterService() : Node("compute_center_service")
    {
        service_ = this->create_service<ComputeCenter>(
                    "compute_center_service",
                    std::bind(&ComputeCenterService::handle_service, this, std::placeholders::_1, std::placeholders::_2)
                    );

        // Sottoscrizione al topic delta_movements_cartesian
        delta_sub_ = this->create_subscription<std_msgs::msg::Float64>(
                    "/delta_movements_cartesian", 10,
                    std::bind(&ComputeCenterService::deltaCallback, this, std::placeholders::_1));

        // TransformBroadcaster per pubblicare le TF
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        RCLCPP_INFO(this->get_logger(), "Compute Service Node ready, waiting for service request...");

    }

    void publishTf(){

        if(is_tf_ready){
        // Costruisci e pubblica la TF
        geometry_msgs::msg::TransformStamped center_transform;
        center_transform.header.stamp = this->now();
        center_transform.header.frame_id = "screw_approach_frame";
        center_transform.child_frame_id = "centered_screw_frame";
        center_transform.transform.translation.x = x_center_;
        center_transform.transform.translation.y = y_center_;
        center_transform.transform.translation.z = 0.0; // Modifica come necessario
        center_transform.transform.rotation.x = 0.0;
        center_transform.transform.rotation.y = 0.0;
        center_transform.transform.rotation.z = 0.0;
        center_transform.transform.rotation.w = 1.0;

        tf_broadcaster_->sendTransform(center_transform);
        }

    }

private:

    void deltaCallback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        // Aggiungi il delta alla lista (mantieni solo gli ultimi 4)
        deltas_.push_back(msg->data);
        if (deltas_.size() > 4) {
            deltas_.erase(deltas_.begin()); // Rimuove il valore più vecchio
        }
    }

    void handle_service(const std::shared_ptr<ComputeCenter::Request> request,
                       const std::shared_ptr<ComputeCenter::Response> response)
    {
        if (deltas_.size() < 4) {
            RCLCPP_WARN(this->get_logger(), "Non ci sono abbastanza delta disponibili.");
            response->success = false; // Ritorna un errore se non ci sono abbastanza dati
            return;
        }

        // Calcola il centro
         x_center_ = (std::abs(deltas_[0] - deltas_[1]))/2 ;
         y_center_ = (std::abs(deltas_[2] - deltas_[3]))/2 ;

        if(deltas_[0] < deltas_[1]){
            x_center_ = - x_center_;
        }

        if(deltas_[2] < deltas_[3]){
            y_center_ = - y_center_;
        }

        is_tf_ready = true;


        RCLCPP_INFO(this->get_logger(), "TF del centro della vite pubblicata");

        response->success = true; // Indica che il calcolo è andato a buon fine
    }

    rclcpp::Service<ComputeCenter>::SharedPtr service_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr delta_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::vector<double> deltas_;
    double x_center_,y_center_;
    bool is_tf_ready=false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ComputeCenterService>();

    // Timer che chiama publishTf() ogni 1000 millisecondi
    auto timer = node->create_wall_timer(
    std::chrono::milliseconds(100),
    [node]() { node->publishTf(); }  // Lambda che richiama la funzione di pubblicazione TF
    );

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

