#include <anritsu_driver/anritsu_driver.hpp>


namespace anritsu_driver
{

void AnritsuDriver::publish(double ch_power, double psd)
{
  if (!chpow_pub_) {
    RCLCPP_ERROR(this->get_logger(), "❌ Publisher not initialized, cannot publish Channel Power!");
    return;
  }

  std_msgs::msg::Float64MultiArray msg;
  msg.data = {ch_power, psd};

  chpow_pub_->publish(msg);

  RCLCPP_DEBUG(this->get_logger(),
    "📊 Channel Power = %.2f dBm | PSD = %.2f dBm/Hz",
    ch_power, psd);
}

void AnritsuDriver::publish(int trace_id, const std::vector<double> &values)
{
  std_msgs::msg::Float64MultiArray msg;
  msg.data = values;

  switch (trace_id)
  {
    case 1:
      if (trace1_pub_) {
        trace1_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(),
                    "📈 Published Trace 1 with %zu samples", values.size());
      }
      break;

    case 2:
      if (trace2_pub_) {
        trace2_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(),
                    "📈 Published Trace 2 with %zu samples", values.size());
      }
      break;

    case 3:
      if (trace3_pub_) {
        trace3_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(),
                    "📈 Published Trace 3 with %zu samples", values.size());
      }
      break;

    case 4:
      if (trace4_pub_) {
        trace4_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(),
                    "📈 Published Trace 4 with %zu samples", values.size());
      }
      break;

    case 5:
      if (trace5_pub_) {
        trace5_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(),
                    "📈 Published Trace 5 with %zu samples", values.size());
      }
      break;

    case 6:
      if (trace6_pub_) {
        trace6_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(),
                    "📈 Published Trace 6 with %zu samples", values.size());
      }
      break;

    default:
      RCLCPP_WARN(this->get_logger(),
                  "Invalid trace ID: %d", trace_id);
      break;
  }
}




}  // namespace anritsu_driver