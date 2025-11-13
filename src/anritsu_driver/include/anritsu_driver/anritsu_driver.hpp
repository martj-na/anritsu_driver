#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <ctime>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <vector>
#include <sstream>
#include <algorithm>




#include <map>
#include <rclcpp/rclcpp.hpp>

#include <dua_node_cpp/dua_node.hpp>
#include <dua_qos_cpp/dua_qos.hpp>

#include <builtin_interfaces/msg/time.hpp>


#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>


#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>

#define UNUSED(arg) (void)(arg)


using namespace builtin_interfaces::msg;


using namespace rcl_interfaces::msg;
using namespace sensor_msgs::msg;
using namespace std_msgs::msg;
using namespace std_srvs::srv;

namespace anritsu_driver
{

class AnritsuDriver : public dua_node::NodeBase
{
public:
  AnritsuDriver(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  virtual ~AnritsuDriver();

private:
  // =====================================================================
  //  Initialization Routines
  // =====================================================================
  void init_parameters() override;
  void init_cgroups() override;
  void init_subscribers() override;
  void init_publishers() override;
  void init_service_servers() override;


  /**
   * @brief Routine to initialize node structures.
   */
  void init_internals();


  // =====================================================================
  //  SCPI Communication Routines
  // =====================================================================
  void scpi_routine();
  void scpi_start();
  void scpi_stop();

  bool scpi_connect();
  void scpi_disconnect();

  bool scpi_send(const std::string &cmd, double wait_s = 0.05);
  std::string scpi_query(const std::string &cmd, double wait_s = 0.05);

   // =====================================================================
  //  Measurements Routines
  // =====================================================================

  void channel_power_routine();
  void zero_span_routine();


  // =====================================================================
  //  Parameters
  // =====================================================================
  std::string configure_mode_ = "";
  bool autostart_ = false;
  bool auto_configure_ = false;

  std::string configure_scpi_ip_ = "";
  int64_t configure_scpi_port_ = 0;

  double configure_analyzer_center_freq_ = 0.0;
  double configure_analyzer_span_ = 0.0;
  double configure_analyzer_rbw_ = 0.0;
  double configure_analyzer_vbw_ = 0.0;
  bool configure_analyzer_if_gain_ = false;
  int64_t configure_analyzer_averages_ = 0;
  int64_t configure_analyzer_reflevel_ = 0;


  
  double configure_analyzer_zerospan_sweep_time_ = 0.0;
  std::string configure_analyzer_zerospan_trigger_source_ = "";

  double configure_analyzer_channelpower_integration_bw_ = 0.0;
 

  struct TraceConfig {
  bool enabled;
  std::string type;
  std::string detector;
  };

  std::vector<TraceConfig> traces_;



  // =====================================================================
  //  Internal state
  // =====================================================================
  int sock_fd_ = -1;
  std::atomic<bool> scpi_running_{false};
  std::thread scpi_thread_;
  std::mutex socket_mutex_;



  
  /* Internal Variables. */
  std::atomic<bool> configure_{false};
  


  // =====================================================================
  /* Subscriptions. */
  rclcpp::Subscription<ByteMultiArray>::SharedPtr input_sub_;

  /* Subscriptions Topics. */
  static const std::string input_sub_topic_;

  /* Subscriptions Callback Groups. */
  rclcpp::CallbackGroup::SharedPtr input_cgroup_;

  /* Subscriptions Callbacks. */

  /**
   * @brief Parse NMEA sentece in String message.
   *
   * @param msg Message to parse.
   */
  void input_clbk(const ByteMultiArray::ConstSharedPtr msg);


    /* Service Servers. */
  rclcpp::Service<SetBool>::SharedPtr enable_srv_;
  rclcpp::Service<Trigger>::SharedPtr configure_srv_;

  /* Service Servers Names. */
  static const std::string enable_srv_name_;
  static const std::string configure_srv_name_;

  
  /* Service Servers Callbacks. */

  /**
   * @brief Enable service callback.
   *
   * @param req Service request.
   * @param res Service response.
   */
  void enable_callback(
    const SetBool::Request::SharedPtr req,
    const SetBool::Response::SharedPtr res);

  /**
   * @brief Initialize as base service callback.
   *
   * @param req Service request.
   * @param res Service response.
   */
  void configure_callback(
    const Trigger::Request::SharedPtr req,
    const Trigger::Response::SharedPtr res);




/* Publishers Routines. */

  /**
  * @brief Publish Channel Power measurement result.
  *
  * @param channel_power  Measured channel power in dBm.
  * @param psd            Measured power spectral density in dBm/Hz.
  */
  void publish(double channel_power, double psd);
  void publish(int trace_id, const std::vector<double> &values);



// =====================================================================
// Publishers
// =====================================================================

// Channel Power
rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr chpow_pub_;    

// Trace Publishers
rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr trace1_pub_;
rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr trace2_pub_;
rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr trace3_pub_;
rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr trace4_pub_;
rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr trace5_pub_;
rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr trace6_pub_;

// =====================================================================
// Publishers Topics
// =====================================================================
static const std::string chpow_pub_topic_;    // Channel Power topic

static const std::string trace1_pub_topic_;
static const std::string trace2_pub_topic_;
static const std::string trace3_pub_topic_;
static const std::string trace4_pub_topic_;
static const std::string trace5_pub_topic_;
static const std::string trace6_pub_topic_;

// =====================================================================
//  Utilities
// =====================================================================
std::vector<double> parse_csv_to_vector(const std::string &csv);
std::string strip_scpi_header(const std::string &input);
std::map<int, std::vector<double>> parse_trace_all_payload(const std::string &payload);

};




}  // namespace anritsu_driver