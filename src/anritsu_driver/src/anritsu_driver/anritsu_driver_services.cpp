
#include <anritsu_driver/anritsu_driver.hpp>

namespace anritsu_driver
{


void AnritsuDriver::enable_callback(
  const SetBool::Request::SharedPtr req,
  const SetBool::Response::SharedPtr res)
{
  if (req->data) {
    // serial_start();
  } else {
    // serial_stop();
  }

  res->set__success(true);
  res->set__message("");
}


void AnritsuDriver::configure_callback(
  const Trigger::Request::SharedPtr req,
  const Trigger::Response::SharedPtr res)
{
  UNUSED(req);
  configure_.store(true, std::memory_order_release);

  res->set__success(true);
  res->set__message("");
}


} // namespace anritsu_driver
