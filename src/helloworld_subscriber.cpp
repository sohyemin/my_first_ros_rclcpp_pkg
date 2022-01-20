#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

//HellowroldSubscriber으로 rclcpp의 Node 클래스 사용하여 사용
class HelloworldSubscriber : public rclcpp::Node
{
public:
  // 클래스 생성자의 정의로 Node클래스의 생성자를 호출하고 이름을 'Helloworld_subscriber'으로 지정
  HelloworldSubscriber(): Node("Helloworld_subscriber")
  {
    //QoS를 설정하여 예기치 못한 상황 발생시, 서브스크라이브 데이터를 버퍼에 10개까지 저장하게 한다.
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    //create_subscription 함수를 사용하여 'helloworld_subscriber_'라는 서브크라이버를 설정한다.
    //퍼블리셔 함수와는 다르게 수신받은 메시지를 처리할 콜백함수를 기입한다.
    helloworld_subscriber_ = this->create_subscription<std_msgs::msg::String>("helloworld",qos_profile,std::bind(&HelloworldSubscriber::subscribe_topic_message,this,_1));
  };

private:
  // 위에서 지정한 콜백함수인 subscribe_topic_message 함수이다.
  // 서브스크라이브한 메시지는 String타입으로 msg라는 이름을 사용하며 받은 메시지를 msg.data에 저장한다.
  // 여기서는 'Hello World: 1'과 같은 메시지를 서브스크라이브하게 된다.
  // RCLCPP_INFO 함수를 이용하여 메시지를 콘솔창에 출력시킨다.
  void subscribe_topic_message(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Received message: '%s'", msg->data.c_str());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr helloworld_subscriber_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc,argv);
  auto node = std::make_shared<HelloworldSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
