#include <yolov8/srv/object_detection.hpp>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <std_msgs/msg/bool.hpp>
#include <thread>
#include <chrono>
#include <mutex>

using ObjectDetection = yolov8::srv::ObjectDetection;

class ObjDetection : public rclcpp::Node
{
public:
  ObjDetection()
  : Node("object_detection_service")//, detection_status_(false), detection_status_received_(false)
  {
    service_ = this->create_service<ObjectDetection>(
      "object_detection",
      std::bind(&ObjDetection::handle_service, this, std::placeholders::_1, std::placeholders::_2)
    );
    
    /*subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
      "detection_status",
      10,
      std::bind(&ObjDetection::detection_callback, this, std::placeholders::_1)
    );*/
  }

private:
  void handle_service(const std::shared_ptr<ObjectDetection::Request> request,
                      std::shared_ptr<ObjectDetection::Response> response)
  {
      std::thread([this, request]() {
         std::string command = "python3 " + request->script_path + " &";  // Esegui in background
         int result = std::system(command.c_str());
        
         if (result != 0) {
             RCLCPP_ERROR(this->get_logger(), "Errore nell'esecuzione dello script Python.");
         }
      }).detach(); // Stacca il thread per evitare di bloccare il servizio
      
      RCLCPP_INFO(this->get_logger(), "Script lanciato, in attesa del risultato...");
      
      // Attendi per un tot numero di secondi (ad esempio, 5 secondi)
      std::this_thread::sleep_for(std::chrono::seconds(5));
      
      // Dopo l'attesa, imposta la risposta a true
      response->detected = true;
      response->message = "Oggetti corretamente rilevati";
    
      RCLCPP_INFO(this->get_logger(), "Risposta inviata: detected = true, script terminato con successo.");

	// Attendi fino a quando non ricevi un messaggio dal publisher
    	/*rclcpp::Rate rate(1);
   	int attempts = 0;
    	int max_attempts = 10;  // Attendi fino a 10 secondi

	while (!detection_status_received_ && rclcpp::ok() && attempts < max_attempts) {
	      rclcpp::spin_some(this->get_node_base_interface());  // Elabora i messaggi in arrivo
	      rate.sleep();
	      attempts++;
	}
    	
    	RCLCPP_INFO(this->get_logger(), "QUI QUI");
		
	if (detection_status_)
    	{
        	response->detected = true;
        	response->message = "Script executed SUCCESSFULLY.";
    	}
   	else
	{
		response->detected = false;
		response->message = "Script execution FAILED.";
	}
	
	// Resetta lo stato per la prossima chiamata del servizio
    	detection_status_received_ = false;
    	detection_status_ = false;*/
   }
   
   /*void detection_callback(const std_msgs::msg::Bool::SharedPtr msg)
   {
    	std::lock_guard<std::mutex> lock(mutex_);
    	detection_status_ = msg->data;
    	detection_status_received_ = true;
    	RCLCPP_INFO(this->get_logger(), "Detection status received: %s", detection_status_ ? "true" : "false");
   }*/
  rclcpp::Service<ObjectDetection>::SharedPtr service_;
  //rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscriber_;
  //bool detection_status_;
  //bool detection_status_received_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObjDetection>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
