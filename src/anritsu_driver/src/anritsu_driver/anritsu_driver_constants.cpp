#include <anritsu_driver/anritsu_driver.hpp>

namespace anritsu_driver
{

/* Subscriptions Topics. */


/* Publishers Topics. */
const std::string AnritsuDriver::chpow_pub_topic_ = "~/channel_power";

const std::string AnritsuDriver::trace1_pub_topic_ = "~/trace1";
const std::string AnritsuDriver::trace2_pub_topic_ = "~/trace2";
const std::string AnritsuDriver::trace3_pub_topic_ = "~/trace3";
const std::string AnritsuDriver::trace4_pub_topic_ = "~/trace4";
const std::string AnritsuDriver::trace5_pub_topic_ = "~/trace5";
const std::string AnritsuDriver::trace6_pub_topic_ = "~/trace6";


/* Service Servers Names. */
const std::string AnritsuDriver::enable_srv_name_ = "~/enable";
const std::string AnritsuDriver::configure_srv_name_ = "~/configure";

} // namespace anritsu_driver
