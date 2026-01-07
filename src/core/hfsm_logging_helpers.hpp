/**
 * @file     hfsm_logging_helpers.hpp
 * @brief    Funciones helper inline para logging estructurado sin macros
 * @author   Lucas C.
 * @date     27/12/2025
 * @version  1.0
 * @license  MIT
 * 
 * @details  Este archivo proporciona funciones helper inline para logging
 *           estructurado sin usar macros, ideal para:
 *           - Evitar conflictos de nombres en headers complejos
 *           - Mejor depuración (puntos de interrupción en funciones reales)
 *           - Mayor flexibilidad y control de tipos
 *           - Uso en templates y contextos donde las macros son problemáticas
 * 
 * @note     Estas funciones son wrappers simples alrededor de HFSMLogger.
 *           Proporcionan la misma funcionalidad que las macros pero con
 *           mejor integración con el sistema de tipos de C++.
 * 
 * @warning  Al ser inline, estas funciones deben definirse en headers.
 *           No añadir lógica compleja para evitar code bloat.
 * 
 * @see      hfsm_logging.hpp - Sistema de logging principal con macros
 */
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
