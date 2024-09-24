#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/srv/tf_publish.hpp>  // Cambia con il tuo servizio
#include <chrono>
#include <thread>

using TfPublisher = tf::srv::TfPublish;

class TfPublisherNode : public rclcpp::Node
{
public:
    TfPublisherNode() : Node("tf_publish_node")
    {
        service_ = this->create_service<TfPublisher>(
        "tf_publish_service",
        std::bind(&TfPublisherNode::handle_service, this, std::placeholders::_1, std::placeholders::_2)
       );

        // TransformBroadcaster per pubblicare le TF
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        RCLCPP_INFO(this->get_logger(), "TF Publisher Node ready, waiting for service request...");

    }
    
    // Funzione periodica per pubblicare le TF
    void publishTf()
    {
            if (!tf_list_.empty()) 
            {
                for (auto &tf : tf_list_)
                {
                    RCLCPP_INFO(this->get_logger(), 
                        "Publishing TF: parent_frame: %s, child_frame: %s, translation: [%f, %f, %f]",
                        tf.header.frame_id.c_str(),
                        tf.child_frame_id.c_str(),
                        tf.transform.translation.x,
                        tf.transform.translation.y,
                        tf.transform.translation.z);
                   
                    // Aggiorna il timestamp con l'ora corrente altrimenti le TF scompaiono in RViz
            	    tf.header.stamp = this->now();
                    
                    tf_broadcaster_->sendTransform(tf);
                    
                    // Pausa breve tra la pubblicazione di ogni TF
    		    //std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
                }
            }
            else
       	   {
            RCLCPP_INFO(this->get_logger(), "No TFs to publish");
           }
    }
    
     // Callback del servizio per gestire la richiesta
    void handle_service(
        const std::shared_ptr<TfPublisher::Request> request,
        std::shared_ptr<TfPublisher::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to publish TFs");

        // Salva le TF dalla richiesta
        tf_list_ = request->tf_list;

        // Risposta di successo
        response->success = true;
    }
    
private:

    rclcpp::Service<TfPublisher>::SharedPtr service_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::vector<geometry_msgs::msg::TransformStamped> tf_list_;
    //rclcpp::TimerBase::SharedPtr timer_;  // Timer per la pubblicazione delle TF
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TfPublisherNode>();
    
    // Timer che chiama publishTf() ogni 1000 millisecondi
    auto timer = node->create_wall_timer(
    std::chrono::milliseconds(100),
    [node]() { node->publishTf(); }  // Lambda che richiama la funzione di pubblicazione TF
    );
    
    /*// Ciclo while per pubblicare periodicamente le TF
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        
        // Pubblica le TF
        node->publishTf();

        // Attendi 1 secondo prima di ripetere il ciclo
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }*/
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

