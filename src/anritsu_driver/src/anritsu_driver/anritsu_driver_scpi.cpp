#include <anritsu_driver/anritsu_driver.hpp>
#include <chrono>
#include <thread>

namespace anritsu_driver
{

// =====================================================================
// SCPI Connection
// =====================================================================
bool AnritsuDriver::scpi_connect()
{
  // Se già connesso, non fare nulla
  if (sock_fd_ >= 0) {
    RCLCPP_WARN(this->get_logger(), "SCPI socket already connected");
    return true;
  }

  // Creazione socket TCP
  sock_fd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (sock_fd_ < 0) {
    RCLCPP_ERROR(this->get_logger(), "Socket creation failed: %s", strerror(errno));
    return false;
  }

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(configure_scpi_port_);

  if (inet_pton(AF_INET, configure_scpi_ip_.c_str(), &addr.sin_addr) <= 0) {
    RCLCPP_ERROR(this->get_logger(), "Invalid IP address: %s", configure_scpi_ip_.c_str());
    close(sock_fd_);
    sock_fd_ = -1;
    return false;
  }

  // Connessione al server SCPI
  if (connect(sock_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to connect to %s:%ld (%s)",
                 configure_scpi_ip_.c_str(), configure_scpi_port_, strerror(errno));
    close(sock_fd_);
    sock_fd_ = -1;
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "✅ Connected to Anritsu on %s:%ld",
              configure_scpi_ip_.c_str(), configure_scpi_port_);
  return true;
}

// =====================================================================
// SCPI Disconnect
// =====================================================================
void AnritsuDriver::scpi_disconnect()
{
  if (sock_fd_ >= 0) {
    close(sock_fd_);
    sock_fd_ = -1;
    RCLCPP_INFO(this->get_logger(), "🔌 SCPI socket closed");
  }
}

// =====================================================================
// SCPI Thread Routine 
// =====================================================================
void AnritsuDriver::scpi_routine()
{
  if (!scpi_connect()) {
    RCLCPP_ERROR(this->get_logger(), "❌ SCPI routine failed to connect");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "🔗 SCPI routine started");

  while (scpi_running_.load(std::memory_order_acquire)) {

    bool expected = true;
    bool config_triggered = configure_.compare_exchange_strong(
      expected, false,
      std::memory_order_release,
      std::memory_order_acquire);

    if (config_triggered) {
      RCLCPP_INFO(this->get_logger(), "⚙️ Configuration trigger detected.");

      //  impostazioni comuni
      scpi_send(":FREQ:CENT " + std::to_string(configure_analyzer_center_freq_) + "Hz");
      scpi_send(":FREQ:SPAN " + std::to_string(configure_analyzer_span_) + "Hz");
      scpi_send(":BAND " + std::to_string(configure_analyzer_rbw_) + "Hz");
      scpi_send(":BAND:VID " + std::to_string(configure_analyzer_vbw_) + "Hz");
      scpi_send(":DISP:WIND:TRAC:Y:SCAL:RLEV " + std::to_string(configure_analyzer_reflevel_) + "dBm");
      if(configure_analyzer_if_gain_){
        scpi_send(":SENS:POW:IF:GAIN:STAT 1");
      }

      if(configure_analyzer_averages_ > 1){
        scpi_send(":SENS:AVER:COUN " + std::to_string(configure_analyzer_averages_));
      }

      for (size_t i = 0; i < traces_.size(); ++i)
      {
        const auto &trace = traces_[i];
        std::string prefix = ":TRAC" + std::to_string(i + 1);

        if (!trace.enabled) {
          RCLCPP_DEBUG(this->get_logger(), "Trace %zu disabled, skipping", i + 1);
          continue;
        }

        scpi_send(prefix + ":TYPE " + trace.type);
        scpi_send(prefix + ":DET:FUNC " + trace.detector);
        scpi_send(prefix + ":DISP 1");
        scpi_send(prefix + ":UPD ON");

        RCLCPP_INFO(this->get_logger(),
                    "Configured Trace %zu: TYPE=%s DET=%s",
                    i + 1, trace.type.c_str(), trace.detector.c_str());
      }

      
      

       
      
      if (configure_mode_ == "channel_power") {
        RCLCPP_INFO(this->get_logger(), "📡 Applying CHANNEL POWER configuration...");

        // impostazioni specifiche per channel power tipo integration bandwith
        scpi_send(":CONFigure:CHPower");
        scpi_send(":SENSe:CHPower:BANDwidth:INT " + std::to_string(configure_analyzer_channelpower_integration_bw_) + "Hz");

      
        channel_power_routine();
      } 
      else if (configure_mode_ == "zero_span") {
        RCLCPP_INFO(this->get_logger(), "📈 Applying ZERO SPAN configuration...");

        // impostazioni specifiche per zero span tipo trigger 

        //  scpi_send("TRIG:SOUR " + configure_analyzer_zerospan_trigger_); //(per ora non lo metto che non mi va di accendere il gps) 

        scpi_send(":FORM:TRAC:DATA ASC");
        scpi_send(":INIT:CONT ON");
        scpi_send(":SWE:TIME " + std::to_string(configure_analyzer_zerospan_sweep_time_) + "s");

        zero_span_routine();
      } 
      else {
        RCLCPP_WARN(this->get_logger(), 
          "⚠️ Unknown analyzer mode: '%s' — skipping configuration.",
          configure_mode_.c_str());
      }
    }

    // Sleep breve per non saturare la CPU
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  scpi_disconnect();
  RCLCPP_INFO(this->get_logger(), "🔌 SCPI routine stopped");
}

// =====================================================================
// Start/Stop Thread
// =====================================================================
void AnritsuDriver::scpi_start()
{
  bool expected = false;
  if (scpi_running_.compare_exchange_strong(expected, true)) {
    scpi_thread_ = std::thread(&AnritsuDriver::scpi_routine, this);
    RCLCPP_INFO(this->get_logger(), "SCPI thread started");
  }
}

void AnritsuDriver::scpi_stop()
{
  bool expected = true;
  if (scpi_running_.compare_exchange_strong(expected, false)) {
    if (scpi_thread_.joinable()) {
      scpi_thread_.join();
    }
    RCLCPP_INFO(this->get_logger(), "SCPI thread stopped");
  }
}



bool AnritsuDriver::scpi_send(const std::string &cmd, double wait_s)
{
  std::lock_guard<std::mutex> lock(socket_mutex_);

  if (sock_fd_ < 0) {
    RCLCPP_ERROR(this->get_logger(), "❌ Cannot send SCPI command: socket not connected");
    return false;
  }

  // Aggiunge newline SCPI
  std::string full_cmd = cmd + "\n";

  ssize_t sent = send(sock_fd_, full_cmd.c_str(), full_cmd.size(), 0);
  if (sent < 0) {
    RCLCPP_ERROR(this->get_logger(), "❌ SCPI send() failed: %s", strerror(errno));
    return false;
  }

  if (static_cast<size_t>(sent) != full_cmd.size()) {
    RCLCPP_WARN(this->get_logger(),
                "⚠️ Partial SCPI command sent (%ld/%zu bytes)",
                sent, full_cmd.size());
  }

  // Piccola attesa opzionale
  if (wait_s > 0) {
    std::this_thread::sleep_for(std::chrono::duration<double>(wait_s));
  }

  RCLCPP_INFO(this->get_logger(), "➡️ Sent SCPI: %s", cmd.c_str());
  return true;
}

// std::string AnritsuDriver::scpi_query(const std::string &cmd, double wait_s)
// {
//   std::lock_guard<std::mutex> lock(socket_mutex_);

//   if (sock_fd_ < 0) {
//     throw std::runtime_error("❌ Cannot query SCPI: socket not connected");
//   }

//   // Invia comando con newline
//   std::string full_cmd = cmd + "\n";
//   ssize_t sent = send(sock_fd_, full_cmd.c_str(), full_cmd.size(), 0);
//   if (sent < 0) {
//     throw std::runtime_error("❌ SCPI send() failed: " + std::string(strerror(errno)));
//   }

//   if (wait_s > 0)
//     std::this_thread::sleep_for(std::chrono::duration<double>(wait_s));

//   // Leggi la risposta
//   char buffer[4096];
//   ssize_t received = recv(sock_fd_, buffer, sizeof(buffer) - 1, 0);
//   if (received < 0) {
//     throw std::runtime_error("❌ SCPI recv() failed: " + std::string(strerror(errno)));
//   }
//   if (received == 0) {
//     throw std::runtime_error("⚠️ SCPI recv(): connection closed by peer");
//   }

//   buffer[received] = '\0';
//   std::string resp(buffer);

//   // Rimuovi CR/LF
//   while (!resp.empty() && (resp.back() == '\n' || resp.back() == '\r')) {
//     resp.pop_back();
//   }

//   RCLCPP_DEBUG(this->get_logger(), "⬅️ SCPI response to '%s': %s", cmd.c_str(), resp.c_str());
//   return resp;
// }
  std::string AnritsuDriver::scpi_query(const std::string &cmd, double wait_s)
  {
    std::lock_guard<std::mutex> lock(socket_mutex_);

    if (sock_fd_ < 0) {
      throw std::runtime_error("Cannot query SCPI: socket not connected");
    }

    // Send command with newline
    const std::string full_cmd = cmd + "\n";
    ssize_t sent = send(sock_fd_, full_cmd.c_str(), full_cmd.size(), 0);
    if (sent < 0) {
      throw std::runtime_error("SCPI send() failed: " + std::string(strerror(errno)));
    }

    if (wait_s > 0.0) {
      std::this_thread::sleep_for(std::chrono::duration<double>(wait_s));
    }

    std::string resp;
    resp.reserve(8192);
    char buf[4096];

    auto append_chunk = [&](ssize_t n) {
      if (n <= 0) {
        if (n == 0) throw std::runtime_error("SCPI recv(): connection closed by peer");
        throw std::runtime_error("SCPI recv() failed: " + std::string(strerror(errno)));
      }
      resp.append(buf, static_cast<size_t>(n));
    };

    // Read first chunk to determine response type
    ssize_t n = recv(sock_fd_, buf, sizeof(buf), 0);
    append_chunk(n);

    if (!resp.empty() && resp[0] == '#') {
      // Definite-length block: "#<n><len><payload>"
      // Make sure we have at least "#<n>".
      while (resp.size() < 2) {
        n = recv(sock_fd_, buf, sizeof(buf), 0);
        append_chunk(n);
      }

      int len_digits = resp[1] - '0';
      if (len_digits <= 0 || len_digits > 9) {
        throw std::runtime_error("Malformed SCPI header: invalid length digit count");
      }

      const size_t header_len = 2u + static_cast<size_t>(len_digits);

      // Ensure we have the full header to parse <len>
      while (resp.size() < header_len) {
        n = recv(sock_fd_, buf, sizeof(buf), 0);
        append_chunk(n);
      }

      size_t payload_len = 0;
      try {
        payload_len = static_cast<size_t>(std::stoul(resp.substr(2, len_digits)));
      } catch (...) {
        throw std::runtime_error("Malformed SCPI header: invalid length value");
      }

      const size_t total_needed = header_len + payload_len; // ignore trailing CR/LF

      // Read until we have at least header + payload
      while (resp.size() < total_needed) {
        n = recv(sock_fd_, buf, sizeof(buf), 0);
        append_chunk(n);
      }

      // Optionally consume a trailing CR/LF if present (do not block if not there)
      // Try one non-blocking peek for a newline; if your socket is blocking only, you can skip this.

    } else {
      // ASCII response: read until newline
      while (resp.empty() || (resp.back() != '\n' && resp.back() != '\r')) {
        n = recv(sock_fd_, buf, sizeof(buf), 0);
        append_chunk(n);
      }
    }

    // Strip trailing CR/LF
    while (!resp.empty() && (resp.back() == '\n' || resp.back() == '\r')) {
      resp.pop_back();
    }

    RCLCPP_DEBUG(this->get_logger(), "SCPI response to '%s': %s", cmd.c_str(), resp.c_str());
    return resp;
  }


}  // namespace anritsu_driver

