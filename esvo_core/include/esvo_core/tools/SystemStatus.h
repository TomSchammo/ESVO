#ifndef ESVO_CORE_TOOLS_SYSTEM_STATUS_H
#define ESVO_CORE_TOOLS_SYSTEM_STATUS_H

#include <optional>
#include <string_view>

namespace esvo_core {

enum class SystemStatus {
  INITIALIZATION,
  WORKING,
  TERMINATE,
  RESET,
};

inline const char* to_string(SystemStatus s) {
  switch (s) {
    case SystemStatus::INITIALIZATION: return "INITIALIZATION";
    case SystemStatus::WORKING:        return "WORKING";
    case SystemStatus::TERMINATE:      return "TERMINATE";
    case SystemStatus::RESET:          return "RESET";
  }
  return "INITIALIZATION";
}

inline std::optional<SystemStatus> system_status_from_string(std::string_view s) {
  if (s == "INITIALIZATION") return SystemStatus::INITIALIZATION;
  if (s == "WORKING")        return SystemStatus::WORKING;
  if (s == "TERMINATE")      return SystemStatus::TERMINATE;
  if (s == "RESET")          return SystemStatus::RESET;
  return std::nullopt;
}
}  // namespace esvo_core

#endif  // ESVO_CORE_TOOLS_SYSTEM_STATUS_H
