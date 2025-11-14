#include <anritsu_driver/anritsu_driver.hpp>
#include <chrono>
#include <thread>

namespace anritsu_driver
{

void AnritsuDriver::channel_power_routine()
{
  RCLCPP_INFO(this->get_logger(), "⚙️ Starting CHANNEL POWER measurement loop...");



  while (scpi_running_.load(std::memory_order_acquire))
  {
    try {
      // Esegui la misura
      std::string response = scpi_query(":READ:CHP?");
      if (response.empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "⚠️ Empty SCPI response from :READ:CHP?");
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        continue;
      }

      // Parsing: response format "channel_power,psd"
      double channel_power = 0.0;
      double psd = 0.0;
      int parsed = sscanf(response.c_str(), "%lf,%lf", &channel_power, &psd);

      if (parsed == 2) {
        publish(channel_power, psd);
      } else {
        RCLCPP_WARN(this->get_logger(),
                    "⚠️ Unexpected CHP response format: '%s'", response.c_str());
      }
    } 
    catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "❌ SCPI query failed: %s", e.what());
      break; // Interrompe il ciclo se la connessione fallisce
    }

   
  }

  RCLCPP_INFO(this->get_logger(), "🟢 CHANNEL POWER routine stopped cleanly.");
}


void AnritsuDriver::zero_span_routine()
{
  RCLCPP_INFO(this->get_logger(), "Executing ZERO SPAN routine...");
  
  const size_t num_traces = 6;
  const size_t points_per_trace = 501;

  // Pre-allocazione container finale
  std::vector<std::vector<double>> all_traces(num_traces);

  while (scpi_running_.load(std::memory_order_acquire))
  {
    try 
    {
      for (size_t i = 0; i < num_traces; ++i)
      {
        const auto &trace = traces_[i];

        if (!trace.enabled) {
          // Traccia disabilitata → padding a zero
          all_traces[i] = std::vector<double>(points_per_trace, 0.0);
          continue;
        }

        // 1️⃣ SCPI query
        scpi_query("*OPC?");
        std::string cmd = ":TRAC:DATA?" + std::to_string(i + 1);
        std::string resp = scpi_query(cmd);

        // 2️⃣ Parsing SCPI CSV
        std::string payload = strip_scpi_header(resp);
        std::vector<double> values = parse_csv_to_vector(payload);

        // 3️⃣ Se la traccia non ha 501 punti → padding a zero
        if (values.size() != points_per_trace) {
          RCLCPP_WARN(this->get_logger(),
                      "Trace %zu wrong size (%zu), padding to %zu",
                      i+1, values.size(), points_per_trace);

          values.resize(points_per_trace, 0.0);
        }

        all_traces[i] = std::move(values);
      }

      // 4️⃣ A questo punto abbiamo sempre 6×501 valori → pubblichiamo
      publish(all_traces);

      std::this_thread::sleep_for(std::chrono::seconds(1));  // 1 Hz
    }
    catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "SCPI query failed: %s", e.what());
      break;
    }
  }

  RCLCPP_INFO(this->get_logger(), "🟢 ZERO SPAN routine stopped.");
}





  
} // namespace anritsu_driver 