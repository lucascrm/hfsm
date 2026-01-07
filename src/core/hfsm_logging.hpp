/**
 * @file     hfsm_logging.hpp
 * @brief    Sistema de logging estructurado y configurable para HFSM
 * @author   Lucas C.
 * @date     27/12/2025
 * @version  1.0
 * @license  MIT
 * 
 * @details  Este archivo define el sistema de logging estructurado del HFSM,
 *           incluyendo:
 *           - Clase HFSMLogger con patron singleton
 *           - Configuración flexible (archivo, consola, niveles, filtros)
 *           - Logging asíncrono con buffer para máximo rendimiento
 *           - Métodos específicos para dominio HFSM
 *           - Macros de conveniencia para uso simplificado
 *           - Integración con sistema de diagnóstico del hardware
 *           - Métricas de rendimiento integradas
 * 
 * @note     Implementa inicialización robusta con fallback a logging síncrono
 *           si el sistema no está inicializado. Usa condition variables para
 *           sincronización eficiente del buffer.
 * 
 * @warning  Las macros de conveniencia deben usarse con cuidado para evitar
 *           conflictos de nombres. No usar en headers que puedan ser incluidos
 *           múltiples veces sin control.
 * 
 * @see      hfsm_logging_helpers.hpp - Funciones helper sin macros
 * @see      hardware/utils/Diagnostics.h - Sistema de diagnóstico base
 */
#ifndef HFSM_LOGGING_HPP
#define HFSM_LOGGING_HPP

#include "hardware/utils/Diagnostics.h"
#include "hfsm_config_core.hpp"
#include "hfsm_error_code.hpp"
#include "hfsm_event_types.hpp"

#include <nlohmann/json.hpp>
#include <chrono>
#include <queue>
#include <fstream>
#include <mutex>
#include <thread>
#include <atomic>
#include <string>
#include <vector>
#include <cstdint>
#include <cstdlib>
#include <condition_variable>

namespace ns_fsm {

// ============================================================================
// CONFIGURACIÓN DE LOG
// ============================================================================

struct LogConfig {
    bool enable_structured_logging;
    bool enable_file_logging;
    bool enable_console_logging;
    size_t max_log_size_mb;
    size_t max_log_files;
    hardware::DiagnosticLevel min_level;
    std::vector<std::string> filtered_subsystems;
    
    LogConfig() 
        : enable_structured_logging(true),
          enable_file_logging(true),
          enable_console_logging(true),
          max_log_size_mb(10),
          max_log_files(5),
          min_level(hardware::DiagnosticLevel::INFO) {}
};

// ============================================================================
// LOGGER PRINCIPAL (Declaración)
// ============================================================================

class HFSMLogger {
private:
    LogConfig config_;
    std::string log_file_path_;
    mutable std::mutex mutex_;
    std::condition_variable buffer_not_empty_;
    std::ofstream log_file_;
    std::atomic<bool> initialized_{false};
    std::atomic<bool> running_{false};
    std::thread flush_thread_;
    std::queue<std::string> log_buffer_;
    size_t current_file_size_{0};
    
    // Singleton - constructor privado
    HFSMLogger();
    ~HFSMLogger();
    
    // No copiable
    HFSMLogger(const HFSMLogger&) = delete;
    HFSMLogger& operator=(const HFSMLogger&) = delete;
    
    // Métodos privados
    void setupFileLogging();
    void startFlushThread();
    void flushWorker();
    void flushToFile();
    void rotateLogFile();
    bool shouldLog(const std::string& subsystem, hardware::DiagnosticLevel level) const;
    
    // Método interno para logging síncrono (inicialización)
    void logDirect(const std::string& subsystem,
                   hardware::DiagnosticLevel level,
                   const std::string& message,
                   const nlohmann::json& context = {});
    
public:
    static HFSMLogger& getInstance();
    
    // API pública
    void initialize(const LogConfig& config = LogConfig());
    void shutdown();
    
    // Métodos de logging estructurado
    void logStructured(const std::string& subsystem,
                      hardware::DiagnosticLevel level,
                      const std::string& message,
                      const nlohmann::json& context = {});
    
    // Métodos específicos para HFSM
    void logStateTransition(uint32_t from_state, uint32_t to_state,
                           const std::string& trigger = "",
                           bool success = true);
    
    void logEventProcessed(const Event& event, bool success = true,
                          const std::string& handler = "");
    
    void logError(ErrorCode code, const std::string& context,
                 const std::string& component = "HFSM");
    
    void logPerformanceMetric(const std::string& metric_name,
                             double value,
                             const std::string& unit = "");
    
    // Configuración
    void setMinLogLevel(hardware::DiagnosticLevel level);
    
    // Estadísticas
    nlohmann::json getStatistics() const;
};

// ============================================================================
// MACROS DE CONVENIENCIA
// ============================================================================

// Macros para logging estructurado
#define HFSM_LOG_TRANSITION(from, to, trigger, success) \
    ns_fsm::HFSMLogger::getInstance().logStateTransition( \
        static_cast<uint32_t>(from), static_cast<uint32_t>(to), \
        trigger, success)

#define HFSM_LOG_EVENT(event, success, handler) \
    ns_fsm::HFSMLogger::getInstance().logEventProcessed(event, success, handler)

#define HFSM_LOG_ERROR(code, context, component) \
    ns_fsm::HFSMLogger::getInstance().logError(code, context, component)

#define HFSM_LOG_PERFORMANCE(name, value, unit) \
    ns_fsm::HFSMLogger::getInstance().logPerformanceMetric(name, value, unit)

// Macro simple para scope (sin problemas de redefinición)
#define HFSM_LOG_SCOPE_START(name) \
    auto hfsm_scope_start_##name = std::chrono::steady_clock::now(); \
    HAL_DEBUG("HFSM", std::string("Entering scope: ") + #name)

#define HFSM_LOG_SCOPE_END(name) \
    do { \
        auto hfsm_scope_end_##name = std::chrono::steady_clock::now(); \
        auto hfsm_scope_duration_##name = std::chrono::duration_cast<std::chrono::milliseconds>( \
            hfsm_scope_end_##name - hfsm_scope_start_##name); \
        ns_fsm::HFSMLogger::getInstance().logPerformanceMetric( \
            "scope_" #name "_duration", hfsm_scope_duration_##name.count(), "ms"); \
        HAL_DEBUG("HFSM", std::string("Exiting scope: ") + #name + \
                 " (took " + std::to_string(hfsm_scope_duration_##name.count()) + "ms)"); \
    } while(0)

} // namespace ns_fsm

#endif // HFSM_LOGGING_HPP
