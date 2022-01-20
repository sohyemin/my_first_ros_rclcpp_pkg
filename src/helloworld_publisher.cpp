// include 및 namespace 구문
// 코드에서 사용되는 std 계열의 헤더를 우선 선언
#include <chrono>
#include <functional>
#include <memory>
#include <string>

// 이어서 rclcpp Node 클래스 사용을 위해 rclcpp.hpp 헤더파일과
// 퍼블리시하는 메시지 타입인 String 메시지 인터페이스를 사용하고자 string.hpp 헤더 파일을 포함시켰다.
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// chrono_literals은 추후에 500ms, 1s처럼 시간을 가식성 높인 문자로 표현하기 위하여 namespace를 사용할 수 있도록 선언하였다.
using namespace std::chrono_literals;

// rclcpp의 Node클래스를 상혹하여 사용할 예정.
class HelloworldPublisher : public rclcpp::Node
{
public:
  HelloworldPublisher()
  // class 생성자의 정의로 Node 클래스의 생성자를 호출하고 해당 노드 이름을 'helloworld_publisher'으로 지정하였다. count_ 변수는 0으로 초기화한다.
  : Node("helloworld_publisher"), count_(0)
  {
    // 퍼블리셔의 서비스 품질 옵션(QoS)을 설정하기 위해 'rclcpp:QoS(rclcpp::KeepLast(10))'과 같이 기본 QoS에서 KeepLast 형태로 'depth'를 '10'으로 설정한다. 이는 통신 상태가 원할하지 못한 상황 등 예기치 못한 경우 퍼블리시할 데이터를 10개까지 버퍼에 저장하라는 설정이다.
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    // create_publisher함수를 이용하여 helloworld_publisher_라는 퍼블리셔를 설정한다.
    // 매개변수로는 토픽에 사용할 토픽 메시지 타입과 토픽이름, QoS 설정을 기입하도록 되어 있다.
    // 토픽 메시지 타입 : String
    // 토픽 이름 : helloworld
    // QoS 설정 : qos_profile
    helloworld_publisher_ = this->create_publisher<std_msgs::msg::String>("helloworld", qos_profile);

    // Node 함수의 create_wall_timer 함수를 이용한 콜백함수 수행 구문
    // 1초마다 지정 콜백함수를 수행하라는 것으로 1초마다 publish_helloworld_msg 함수를 수행한다.
    timer_ = this->create_wall_timer(1s, std::bind(&HelloworldPublisher::publish_helloworld_msg,this));
  }

  private:
    void publish_helloworld_msg()
    {
      //msg 함수를 String 타입으로 선언
      auto msg = std_msgs::msg::String();
      //보낼 메시지는 msg.data에 저장하게 된다.
      // 콜백함수가 실행될 때마다 1씩 증가하는 count_ 값을 문자열에 포함시켜 publish 함수를 통해 퍼블리시하게 된다.
      msg.data = "Hello World: "+std::to_string(count_++);

      // RCLCPP_INFO함수는 콘솔창에 출력시키는 함수로 5가지 종류가 존재
      // 일반적인 정보 전달에는 RCLCPP_INFO함수가 사용되며 현재 퍼블리시되는 메시지를 콘솔창에 출력하는 구문을 마지막에 넣어주었다.
      RCLCPP_INFO(this->get_logger(), "Published message: '%s", msg.data.c_str());
      helloworld_publisher_->publish(msg);
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr helloworld_publisher_;
    size_t count_;
};


// main함수에서 rclcpp::init을 초기화하고
// 위에서 작성한 HelloworldPublisher 클래스를 node라는 이름으로 생성한다.
// 노드를 spin시켜 콜백함수가 실행될 수 있도록하며 'Ctrl + c'와 같은 인터럽트 시그널 예외 상황에서는 rclcpp::shutdown 함수로 노드를 소멸하고 프로세스를 종료하게 된다.
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HelloworldPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
