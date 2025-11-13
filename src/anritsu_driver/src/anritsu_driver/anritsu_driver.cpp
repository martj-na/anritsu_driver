#include <anritsu_driver/anritsu_driver.hpp>


namespace anritsu_driver
{



AnritsuDriver::AnritsuDriver(const rclcpp::NodeOptions & opts)
: NodeBase("anritsu_driver", opts, true)
{
  dua_init_node();

  init_internals();

  RCLCPP_INFO(this->get_logger(), "Node initialized");
}


AnritsuDriver::~AnritsuDriver()
{
  scpi_stop();
  scpi_disconnect();
}


void AnritsuDriver::init_cgroups()
{
  input_cgroup_ = dua_create_exclusive_cgroup();
}



void AnritsuDriver::init_subscribers()
{
  // if(enable_input_) {
  //   input_sub_ = dua_create_subscription<ByteMultiArray>(
  //     input_sub_topic_,
  //     std::bind(
  //       &AnritsuDriver::input_clbk,
  //       this,
  //       std::placeholders::_1),
  //     dua_qos::Reliable::get_datum_qos(),
  //     input_cgroup_);
}



void AnritsuDriver::init_publishers()
{
  chpow_pub_ = dua_create_publisher<std_msgs::msg::Float64MultiArray>(
    chpow_pub_topic_,
    dua_qos::Reliable::get_datum_qos());

  trace1_pub_ = dua_create_publisher<std_msgs::msg::Float64MultiArray>(
    trace1_pub_topic_,
    dua_qos::Reliable::get_datum_qos());
  trace2_pub_ = dua_create_publisher<std_msgs::msg::Float64MultiArray>(
    trace2_pub_topic_,
    dua_qos::Reliable::get_datum_qos());  
  
  trace3_pub_ = dua_create_publisher<std_msgs::msg::Float64MultiArray>(
    trace3_pub_topic_,      
    dua_qos::Reliable::get_datum_qos());
  
  trace4_pub_ = dua_create_publisher<std_msgs::msg::Float64MultiArray>(
    trace4_pub_topic_,      
    dua_qos::Reliable::get_datum_qos());
  trace5_pub_ = dua_create_publisher<std_msgs::msg::Float64MultiArray>(
    trace5_pub_topic_,
    dua_qos::Reliable::get_datum_qos());
  trace6_pub_ = dua_create_publisher<std_msgs::msg::Float64MultiArray>(
    trace6_pub_topic_,
    dua_qos::Reliable::get_datum_qos());

}


void AnritsuDriver::init_service_servers()
{
  enable_srv_ = dua_create_service_server<SetBool>(
    enable_srv_name_,
    std::bind(
      &AnritsuDriver::enable_callback,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  configure_srv_ = dua_create_service_server<Trigger>(
    configure_srv_name_,
    std::bind(
      &AnritsuDriver::configure_callback,
      this,
      std::placeholders::_1,
      std::placeholders::_2));
}



void AnritsuDriver::init_internals()
{
  configure_.store(auto_configure_, std::memory_order_release);
  if (autostart_) {
    scpi_start();
  }

  traces_.clear();
  RCLCPP_INFO(this->get_logger(), "Initializing trace configurations...");

  for (int i = 1; i <= 6; ++i)
  {
    TraceConfig t;
    std::string prefix = "configure.analyzer.traces.trace" + std::to_string(i) + ".";

    // Read ROS2 parameters
    this->get_parameter(prefix + "enabled", t.enabled);
    this->get_parameter(prefix + "type", t.type);
    this->get_parameter(prefix + "detector", t.detector);

    traces_.push_back(t);

    // Log the result for each trace
    RCLCPP_INFO(this->get_logger(),
                "Trace %d -> enabled: %s | type: %s | detector: %s",
                i,
                t.enabled ? "true" : "false",
                t.type.c_str(),
                t.detector.c_str());
  }

  // Summary log
  RCLCPP_INFO(this->get_logger(), "Total traces loaded: %zu", traces_.size());
}

} // namespace anritsu_driver



#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(anritsu_driver::AnritsuDriver)







