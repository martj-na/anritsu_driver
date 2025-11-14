#include <anritsu_driver/anritsu_driver.hpp>

namespace anritsu_driver
{

/* Subscriptions Topics. */


/* Publishers Topics. */
const std::string AnritsuDriver::chpow_pub_topic_ = "~/channel_power";

const std::string AnritsuDriver::traces_pub_topic_ = "~/traces";



/* Service Servers Names. */
const std::string AnritsuDriver::enable_srv_name_ = "~/enable";
const std::string AnritsuDriver::configure_srv_name_ = "~/configure";

} // namespace anritsu_driver
