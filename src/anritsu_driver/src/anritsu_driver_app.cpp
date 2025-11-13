/**
 * Anritsu Driver standalone application.
 *
 * Author: <tuo nome o "dotX Automation">
 * Date: November 2025
 */

/**
 * Copyright 2025 dotX Automation s.r.l.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <cstdlib>
#include <csignal>

#include <rclcpp/rclcpp.hpp>

#include <ros2_app_manager/ros2_app_manager.hpp>
#include <ros2_signal_handler/ros2_signal_handler.hpp>

#include <anritsu_driver/anritsu_driver.hpp>

using namespace dua_app_management;

int main(int argc, char ** argv)
{
  // =====================================================================
  //  Application Manager initialization
  // =====================================================================
  ROS2AppManager<rclcpp::executors::SingleThreadedExecutor,
    anritsu_driver::AnritsuDriver> app_manager(
    argc,
    argv,
    "anritsu_driver_app");

  // =====================================================================
  //  Setup system signal handler for safe shutdown
  // =====================================================================
  SignalHandler & sig_handler = SignalHandler::get_global_signal_handler();
  sig_handler.init(
    app_manager.get_context(),
    "anritsu_driver_signal_handler",
    app_manager.get_executor());

  sig_handler.install(SIGINT);
  sig_handler.install(SIGTERM);
  sig_handler.install(SIGQUIT);
  sig_handler.ignore(SIGHUP);
  sig_handler.ignore(SIGUSR1);
  sig_handler.ignore(SIGUSR2);

  // =====================================================================
  //  Run the node
  // =====================================================================
  app_manager.run();

  // =====================================================================
  //  Graceful shutdown
  // =====================================================================
  app_manager.shutdown();
  sig_handler.fini();

  return EXIT_SUCCESS;
}