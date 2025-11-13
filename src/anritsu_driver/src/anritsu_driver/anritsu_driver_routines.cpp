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
// questa va bene a patto che la frequenza di campionamento in questa funzione è pari alla frequenza del trigger 
// legge le tracce singolarmente, il che significa che se il trigger è troppo veloce e lo strumento aggiorna le tracce troppo velocemente, potrebbe succedere che le tracce raccolte da questo software non siano ben sincronizzate e appartengono a sweep successivi 
// per risolvere sto problema si deve implementare un parser dito al culo di TRACE:DATA:ALL che legge tutte le tracce in un colpo solo
{
  RCLCPP_INFO(this->get_logger(), "Executing ZERO SPAN routine...");
  
  while (scpi_running_.load(std::memory_order_acquire))
  {
    try {
      for (size_t i = 0; i < traces_.size(); ++i)
      {
        const auto &trace = traces_[i];
        if (!trace.enabled) {
          RCLCPP_DEBUG(this->get_logger(),
                       "Trace %zu disabled, skipping query for data", i + 1);
          continue;
        }
        
      
        // ============================================
        // 1️⃣ Trace Acquisition
        // ============================================
        scpi_query("*OPC?");
        std::string cmd = ":TRAC:DATA?" + std::to_string(i + 1);
        std::string resp = scpi_query(cmd);
        RCLCPP_DEBUG(this->get_logger(),
                     "Trace %zu: %zu bytes received", i + 1, resp.size());

        // ============================================
        // 2️⃣ Parsing Data
        // ============================================
        std::string payload = strip_scpi_header(resp);
        std::vector<double> values = parse_csv_to_vector(payload);

        // ============================================
        // 3️⃣ Publish Data
        // ============================================
        publish(static_cast<int>(i + 1), values);

        
      }

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