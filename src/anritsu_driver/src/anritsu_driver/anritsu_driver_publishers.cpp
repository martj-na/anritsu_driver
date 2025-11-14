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



  void AnritsuDriver::publish(const std::vector<std::vector<double>> &all_traces)
  {
    if (!traces_pub_) {
      RCLCPP_ERROR(this->get_logger(),
                  "ZeroSpan publisher not initialized");
      return;
    }

    const size_t num_traces = 6;
    const size_t points_per_trace = 501;

    std_msgs::msg::Float64MultiArray msg;

    // Layout 2D: [6 traces, 501 points]
    msg.layout.dim.resize(2);

    msg.layout.dim[0].label  = "traces";
    msg.layout.dim[0].size   = num_traces;
    msg.layout.dim[0].stride = num_traces * points_per_trace;

    msg.layout.dim[1].label  = "points";
    msg.layout.dim[1].size   = points_per_trace;
    msg.layout.dim[1].stride = points_per_trace;

    msg.data.reserve(num_traces * points_per_trace);

    for (size_t i = 0; i < num_traces; ++i)
    {
      const auto &trace = all_traces[i];

      if (trace.size() == points_per_trace) {
        msg.data.insert(msg.data.end(), trace.begin(), trace.end());
      } else {
        // Qualsiasi errore/empty → padding
        msg.data.insert(msg.data.end(), points_per_trace, 0.0);
      }
    }

    traces_pub_->publish(msg);

    RCLCPP_INFO(this->get_logger(),
                "📤 Published ZeroSpan RAW (6×501 samples = %zu values)",
                msg.data.size());
  }

}  // namespace anritsu_driver