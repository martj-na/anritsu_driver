
#include <anritsu_driver/anritsu_driver.hpp>

namespace anritsu_driver
{

// bool AnritsuDriver::validate_configure_mode(const rclcpp::Parameter & p)
// {
//   std::string rtk_mode = p.as_string();

//   if (mode == "disabled") {
//     configure_rtk_mode_ = MeasurementMode::Disabled;
//   } else if (rtk_mode == "ZeroSpan") {
//     configure_rtk_mode_ = MeasurementMode::ZeroSpan;
//   } else if (rtk_mode == "ChannelPower") {
//     configure_rtk_mode_ = MeasurementMode::ChannelPower;
//   } else {
//     RCLCPP_ERROR(
//       this->get_logger(),
//       "AnritsuDriver::validate_configure_rtk_mode: Invalid configure.rtk.mode parameter: %s",
//       rtk_mode.c_str());
//     return false;
//   }

//   return true;
// }

} // namespace anritsu_driver