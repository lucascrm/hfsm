// hfsm_logging_helpers.hpp - FUNCIONES HELPER PARA LOGGING (sin macros)
#ifndef HFSM_LOGGING_HELPERS_HPP
#define HFSM_LOGGING_HELPERS_HPP

#include "hfsm_logging.hpp"

namespace ns_fsm {

// Funciones helper inline para logging
inline void log_debug_helper(const std::string& subsystem, 
                            const std::string& message,
                            const nlohmann::json& context = {}) {
    HFSMLogger::getInstance().logStructured(
        subsystem,
        hardware::DiagnosticLevel::DEBUG,
        message,
        context
    );
}

inline void log_info_helper(const std::string& subsystem,
                           const std::string& message,
                           const nlohmann::json& context = {}) {
    HFSMLogger::getInstance().logStructured(
        subsystem,
        hardware::DiagnosticLevel::INFO,
        message,
        context
    );
}

inline void log_warn_helper(const std::string& subsystem,
                           const std::string& message,
                           const nlohmann::json& context = {}) {
    HFSMLogger::getInstance().logStructured(
        subsystem,
        hardware::DiagnosticLevel::WARN,
        message,
        context
    );
}

inline void log_error_helper(const std::string& subsystem,
                            const std::string& message,
                            const nlohmann::json& context = {}) {
    HFSMLogger::getInstance().logStructured(
        subsystem,
        hardware::DiagnosticLevel::ERROR,
        message,
        context
    );
}

inline void log_fatal_helper(const std::string& subsystem,
                            const std::string& message,
                            const nlohmann::json& context = {}) {
    HFSMLogger::getInstance().logStructured(
        subsystem,
        hardware::DiagnosticLevel::FATAL,
        message,
        context
    );
}

} // namespace ns_fsm

#endif // HFSM_LOGGING_HELPERS_HPP
