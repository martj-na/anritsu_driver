#include <anritsu_driver/anritsu_driver.hpp>



namespace anritsu_driver
{

std::string AnritsuDriver::strip_scpi_header(const std::string &input)
{
  if (input.empty() || input[0] != '#')
    return input;

  if (input.size() < 2)
    throw std::runtime_error("Malformed SCPI header: too short");

  int len_digits = input[1] - '0';
  if (len_digits <= 0 || len_digits > 9)
    throw std::runtime_error("Malformed SCPI header: invalid length digit count");

  if (input.size() < static_cast<size_t>(2 + len_digits))
    throw std::runtime_error("Malformed SCPI header: incomplete header");

  // Compute start index of payload
  size_t start_idx = 2 + len_digits;
  if (input.size() <= start_idx)
    return "";

  std::string payload = input.substr(start_idx);

  while (!payload.empty() && (payload.back() == '\n' || payload.back() == '\r'))
    payload.pop_back();

  return payload;
}

std::vector<double> AnritsuDriver::parse_csv_to_vector(const std::string &csv)
{
  std::vector<double> result;
  result.reserve(1024);  // Preallocazione per efficienza (opzionale)

  std::stringstream ss(csv);
  std::string token;

  while (std::getline(ss, token, ','))
  {
    // Rimuove eventuali spazi o caratteri di controllo
    token.erase(std::remove_if(token.begin(), token.end(),
                               [](unsigned char c) { return std::isspace(c); }),
                token.end());

    if (token.empty())
      continue;

    try {
      result.push_back(std::stod(token));
    } catch (...) {
      // Ignora token non numerici (es. stringhe residue)
    }
  }

  return result;
}


// std::map<int, std::vector<double>>
// AnritsuDriver::parse_trace_all_payload(const std::string &payload)
// {
//   std::map<int, std::vector<double>> traces;

//   if (payload.size() < 4)
//     throw std::runtime_error("TRACE:DATA:ALL payload too short");

//   const char* data = payload.data();

//   // Header: [num_points][flags]
//   uint16_t num_points = 0;
//   uint16_t flags = 0;
//   std::memcpy(&num_points, data, sizeof(uint16_t));
//   std::memcpy(&flags, data + 2, sizeof(uint16_t));

// #if __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
//   num_points = (num_points >> 8) | (num_points << 8);
//   flags = (flags >> 8) | (flags << 8);
// #endif

//   if (num_points == 0) {
//     RCLCPP_WARN(rclcpp::get_logger("anritsu_driver"),
//                 "TRACE:DATA:ALL reports zero points, skipping parse");
//     return traces;
//   }

//   const size_t bytes_per_trace = num_points * sizeof(double);
//   const char* ptr = data + 4;
//   size_t remaining = payload.size() - 4;

//   int trace_id = 1;
//   for (int bit = 0; bit < 16; ++bit)
//   {
//     if (!(flags & (1 << bit)))
//       continue;  // trace disabilitata

//     if (remaining < bytes_per_trace) {
//       RCLCPP_WARN(rclcpp::get_logger("anritsu_driver"),
//                   "TRACE:DATA:ALL payload truncated while parsing trace %d", trace_id);
//       break;
//     }

//     std::vector<double> values(num_points);
//     std::memcpy(values.data(), ptr, bytes_per_trace);

// #if __BYTE_ORDER__ != __ORDER_LITTLE_ENDIAN__
//     for (auto &v : values) {
//       uint64_t u;
//       std::memcpy(&u, &v, sizeof(u));
//       u = ((u & 0x00000000000000FFULL) << 56) |
//           ((u & 0x000000000000FF00ULL) << 40) |
//           ((u & 0x0000000000FF0000ULL) << 24) |
//           ((u & 0x00000000FF000000ULL) << 8 ) |
//           ((u & 0x000000FF00000000ULL) >> 8 ) |
//           ((u & 0x0000FF0000000000ULL) >> 24) |
//           ((u & 0x00FF000000000000ULL) >> 40) |
//           ((u & 0xFF00000000000000ULL) >> 56);
//       std::memcpy(&v, &u, sizeof(u));
//     }
// #endif

//     traces[trace_id] = std::move(values);
//     ptr += bytes_per_trace;
//     remaining -= bytes_per_trace;
//     trace_id++;
//   }

//   return traces;
// }

} // namespace anritsu_driver



