// hfsm_event.cpp - IMPLEMENTACIÓN DE EVENTOS Y FUNCIONES DE UTILIDAD
#include "hfsm_event_types.hpp"

#include <algorithm>
#include <sstream>
#include <iomanip>
#include <nlohmann/json.hpp>

namespace ns_fsm {

// ============================================================================
// IMPLEMENTACIÓN DE CONSTRUCTORES DE EVENT
// ============================================================================

Event::Event() : type(EventType::UNKNOWN), 
                 timestamp(std::chrono::system_clock::now()),
                 source_module("system"),
                 source_component("unknown") {}

Event::Event(EventType t, const std::string& source)
    : type(t), timestamp(std::chrono::system_clock::now()),
      source_module(source), source_component("main") {}

Event::Event(EventType t, EventData d, const std::string& source)
    : type(t), data(std::move(d)), 
      timestamp(std::chrono::system_clock::now()),
      source_module(source), source_component("main") {}

Event::Event(EventType t, const char* msg, const std::string& src)
    : type(t), data(std::string(msg)),
      timestamp(std::chrono::system_clock::now()),
      source_module(src), source_component("main") {}

Event::Event(EventType t, int value, const std::string& src)
    : type(t), data(value),
      timestamp(std::chrono::system_clock::now()),
      source_module(src), source_component("main") {}

Event::Event(EventType t, double value, const std::string& src)
    : type(t), data(value),
      timestamp(std::chrono::system_clock::now()),
      source_module(src), source_component("main") {}

Event::Event(EventType t, bool value, const std::string& src)
    : type(t), data(value),
      timestamp(std::chrono::system_clock::now()),
      source_module(src), source_component("main") {}

// ============================================================================
// IMPLEMENTACIÓN DE MÉTODOS DE EVENT
// ============================================================================

bool Event::hasData() const {
    return !std::holds_alternative<std::monostate>(data);
}

bool Event::isError() const {
    return type == EventType::SYSTEM_ERROR || 
           type == EventType::MODULE_ERROR ||
           type == EventType::HARDWARE_FAULT ||
           type == EventType::TASK_FAILED ||
           type == EventType::RECOVERY_FAILED;
}

bool Event::isHighPriority() const {
    return priority >= 2 || 
           type == EventType::EMERGENCY_STOP ||
           type == EventType::CRITICAL_BATTERY ||
           type == EventType::EMERGENCY_BUTTON_PRESSED;
}

std::string Event::toString() const {
    std::string type_str = eventTypeToString(type);
    
    // Convertir timestamp a string legible
    auto time_t = std::chrono::system_clock::to_time_t(timestamp);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");
    
    return "Event[" + type_str + "] from " + source_module + 
           " at " + ss.str() + " seq:" + std::to_string(sequence_number);
}

nlohmann::json Event::toJson() const {
    nlohmann::json j;
    j["type"] = static_cast<uint32_t>(type);
    j["timestamp"] = std::chrono::duration_cast<std::chrono::milliseconds>(
        timestamp.time_since_epoch()).count();
    j["source_module"] = source_module;
    j["source_component"] = source_component;
    j["sequence_number"] = sequence_number;
    j["correlation_id"] = correlation_id;
    j["priority"] = priority;
    j["requires_acknowledgment"] = requires_acknowledgment;
    j["metadata"] = metadata;
    j["has_data"] = hasData();
    
    // Intentar incluir datos si están disponibles
    if (hasData()) {
        try {
            if (std::holds_alternative<std::string>(data)) {
                j["data"] = std::get<std::string>(data);
            } else if (std::holds_alternative<int32_t>(data)) {
                j["data"] = std::get<int32_t>(data);
            } else if (std::holds_alternative<double>(data)) {
                j["data"] = std::get<double>(data);
            } else if (std::holds_alternative<bool>(data)) {
                j["data"] = std::get<bool>(data);
            } else if (std::holds_alternative<nlohmann::json>(data)) {
                j["data"] = std::get<nlohmann::json>(data);
            }
        } catch (...) {
            // No serializar datos complejos
        }
    }
    
    return j;
}

// ============================================================================
// INSTANCIACIONES EXPLÍCITAS DE PLANTILLAS
// ============================================================================

template std::optional<std::string> Event::getDataAs<std::string>() const;
template std::optional<int32_t> Event::getDataAs<int32_t>() const;
template std::optional<double> Event::getDataAs<double>() const;
template std::optional<bool> Event::getDataAs<bool>() const;
template std::optional<nlohmann::json> Event::getDataAs<nlohmann::json>() const;

// ============================================================================
// FUNCIONES DE UTILIDAD
// ============================================================================

std::string eventTypeToString(EventType type) {
    switch (type) {
        // Control del sistema
        case EventType::SYSTEM_START: return "SYSTEM_START";
        case EventType::SYSTEM_SHUTDOWN: return "SYSTEM_SHUTDOWN";
        case EventType::SYSTEM_ERROR: return "SYSTEM_ERROR";
        case EventType::SYSTEM_RECOVERY: return "SYSTEM_RECOVERY";
        case EventType::SYSTEM_READY: return "SYSTEM_READY";
        case EventType::SYSTEM_CHECK_RESULT: return "SYSTEM_CHECK_RESULT";
        
        // Shutdown
        case EventType::SHUTDOWN_REQUEST: return "SHUTDOWN_REQUEST";
        case EventType::SHUTDOWN_INITIATED: return "SHUTDOWN_INITIATED";
        case EventType::SHUTDOWN_IMMEDIATE: return "SHUTDOWN_IMMEDIATE";
        case EventType::SHUTDOWN_PREPARE: return "SHUTDOWN_PREPARE";
        case EventType::SHUTDOWN_COMMAND: return "SHUTDOWN_COMMAND";
        case EventType::SHUTDOWN_TIMEOUT_WARNING: return "SHUTDOWN_TIMEOUT_WARNING";
        
        // Control básico
        case EventType::PAUSE: return "PAUSE";
        case EventType::RESUME: return "RESUME";
        
        // Estados HFSM
        case EventType::STATE_CHANGED: return "STATE_CHANGED";
        case EventType::STATE_COMPLETED: return "STATE_COMPLETED";
        case EventType::STATE_FAILED: return "STATE_FAILED";
        
        // Red
        case EventType::NETWORK_COMMAND: return "NETWORK_COMMAND";
        case EventType::NETWORK_TELEOP: return "NETWORK_TELEOP";
        case EventType::NETWORK_DISCONNECTED: return "NETWORK_DISCONNECTED";
        
        // Respuesta a red
        case EventType::NETWORK_STATE_UPDATE: return "NETWORK_STATE_UPDATE";
        case EventType::NETWORK_TASK_RESULT: return "NETWORK_TASK_RESULT";
        case EventType::NETWORK_ERROR: return "NETWORK_ERROR";
        case EventType::TELEOP_REQUEST_RECEIVED: return "TELEOP_REQUEST_RECEIVED";
        case EventType::AUTONOMOUS_REQUEST_RECEIVED: return "AUTONOMOUS_REQUEST_RECEIVED";
        
        // Módulos
        case EventType::MODULE_INITIALIZED: return "MODULE_INITIALIZED";
        case EventType::MODULE_SHUTDOWN: return "MODULE_SHUTDOWN";
        case EventType::MODULE_ERROR: return "MODULE_ERROR";
        
        // Seguridad
        case EventType::SECURITY_AUTH_REQUEST: return "SECURITY_AUTH_REQUEST";
        case EventType::SECURITY_AUTH_SUCCESS: return "SECURITY_AUTH_SUCCESS";
        case EventType::SECURITY_AUTH_FAILED: return "SECURITY_AUTH_FAILED";
        
        // Hardware
        case EventType::HARDWARE_FAULT: return "HARDWARE_FAULT";
        
        // Batería
        case EventType::LOW_BATTERY: return "LOW_BATTERY";
        case EventType::CRITICAL_BATTERY: return "CRITICAL_BATTERY";
        
        // Monitoreo de sistema
        case EventType::HIGH_CPU_USAGE: return "HIGH_CPU_USAGE";
        case EventType::HIGH_TEMPERATURE: return "HIGH_TEMPERATURE";
        
        // Modos de operación
        case EventType::MODE_AUTONOMOUS: return "MODE_AUTONOMOUS";
        case EventType::MODE_TELEOP: return "MODE_TELEOP";
        
        // Diagnóstico
        case EventType::DIAGNOSTICS_START: return "DIAGNOSTICS_START";
        case EventType::DIAGNOSTICS_COMPLETE: return "DIAGNOSTICS_COMPLETE";
        case EventType::DIAGNOSTICS_FAILED: return "DIAGNOSTICS_FAILED";
        
        // Emergencia
        case EventType::EMERGENCY_STOP: return "EMERGENCY_STOP";
        case EventType::EMERGENCY_BUTTON_PRESSED: return "EMERGENCY_BUTTON_PRESSED";
        case EventType::EMERGENCY_CLEARED: return "EMERGENCY_CLEARED";
        
        // Configuración
        case EventType::CONFIG_LOADED: return "CONFIG_LOADED";
        case EventType::CONFIG_CHANGED: return "CONFIG_CHANGED";
        case EventType::CONFIG_ERROR: return "CONFIG_ERROR";
        
        // Logging
        case EventType::LOG_CRITICAL: return "LOG_CRITICAL";
        case EventType::LOG_ERROR: return "LOG_ERROR";
        case EventType::LOG_WARNING: return "LOG_WARNING";
        case EventType::LOG_INFO: return "LOG_INFO";
        case EventType::LOG_DEBUG: return "LOG_DEBUG";
        
        // Errores genéricos
        case EventType::CRITICAL_ERROR: return "CRITICAL_ERROR";
        case EventType::TEMPORARY_FAILURE: return "TEMPORARY_FAILURE";
        case EventType::COMMUNICATION_ERROR: return "COMMUNICATION_ERROR";
        
        // Velocidad
        case EventType::SPEED_LIMIT_CHANGED: return "SPEED_LIMIT_CHANGED";
        
        // Auditoría
        case EventType::AUDIT_EVENT: return "AUDIT_EVENT";
        case EventType::AUDIT_SECURITY_VIOLATION: return "AUDIT_SECURITY_VIOLATION";
        
        // Performance
        case EventType::PERFORMANCE_METRIC: return "PERFORMANCE_METRIC";
        case EventType::PERFORMANCE_ALERT: return "PERFORMANCE_ALERT";
        
        // Limpieza
        case EventType::CLEANUP_STARTED: return "CLEANUP_STARTED";
        case EventType::CLEANUP_COMPLETE: return "CLEANUP_COMPLETE";
        case EventType::CLEANUP_FAILED: return "CLEANUP_FAILED";
        
        // Navegación
        case EventType::NAV_GOAL_RECEIVED: return "NAV_GOAL_RECEIVED";
        case EventType::NAV_GOAL_REACHED: return "NAV_GOAL_REACHED";
        case EventType::NAV_OBSTACLE_DETECTED: return "NAV_OBSTACLE_DETECTED";
        case EventType::NAV_PATH_BLOCKED: return "NAV_PATH_BLOCKED";
        case EventType::NAV_EMERGENCY_STOP: return "NAV_EMERGENCY_STOP";
        case EventType::NAV_LOCALIZATION_LOST: return "NAV_LOCALIZATION_LOST";
        case EventType::NAV_LOCALIZATION_RECOVERED: return "NAV_LOCALIZATION_RECOVERED";
        
        // SLAM
        case EventType::LOCALIZATION_LOW_CONFIDENCE: return "LOCALIZATION_LOW_CONFIDENCE";
        case EventType::LOCALIZATION_SUCCESS: return "LOCALIZATION_SUCCESS";
        case EventType::LOCALIZATION_FAILED: return "LOCALIZATION_FAILED";
        
        // Tareas
        case EventType::TASK_START: return "TASK_START";
        case EventType::TASK_COMPLETE: return "TASK_COMPLETE";
        case EventType::TASK_FAILED: return "TASK_FAILED";
        
        // Recuperación
        case EventType::RECOVERY_SUCCESS: return "RECOVERY_SUCCESS";
        case EventType::RECOVERY_FAILED: return "RECOVERY_FAILED";
        case EventType::RECOVERY_REQUEST: return "RECOVERY_REQUEST";
        
        // Watchdog
        case EventType::WATCHDOG_TIMEOUT: return "WATCHDOG_TIMEOUT";
        
        // Timer
        case EventType::TIMER_TICK: return "TIMER_TICK";
        
        // Teleoperación
        case EventType::TELEOP_STATE_ENTER: return "TELEOP_STATE_ENTER";
        case EventType::TELEOP_STATE_EXIT: return "TELEOP_STATE_EXIT";
        case EventType::TELEOP_COMMAND: return "TELEOP_COMMAND";
        case EventType::RETURN_TO_IDLE: return "RETURN_TO_IDLE";
        
        // Estados autónomos
        case EventType::AUTONOMOUS_STATE_ENTER: return "AUTONOMOUS_STATE_ENTER";
        case EventType::AUTONOMOUS_STATE_EXIT: return "AUTONOMOUS_STATE_EXIT";
        
        // Misiones
        case EventType::MISSION_STARTED: return "MISSION_STARTED";
        case EventType::MISSION_RECEIVED: return "MISSION_RECEIVED";
        case EventType::MISSION_COMPLETE: return "MISSION_COMPLETE";
        case EventType::MISSION_ABORT: return "MISSION_ABORT";
        case EventType::MAP_SAVE_REQUEST: return "MAP_SAVE_REQUEST";
        case EventType::EXPLORATION_COMPLETE: return "EXPLORATION_COMPLETE";
        case EventType::MANUAL_RESET: return "MANUAL_RESET";
        case EventType::WAYPOINT_REACHED: return "WAYPOINT_REACHED";
        
        // Salud del sistema
        case EventType::HEARTBEAT: return "HEARTBEAT";
        case EventType::KEEPALIVE: return "KEEPALIVE";
        case EventType::SYSTEM_HEALTH_STATUS: return "SYSTEM_HEALTH_STATUS";
        
        default: 
            return "UNKNOWN(" + std::to_string(static_cast<uint32_t>(type)) + ")";
    }
}

std::string stateTypeToString(StateType type) {
    switch (type) {
        case StateType::UNINITIALIZED: return "UNINITIALIZED";
        case StateType::INITIALIZING: return "INITIALIZING";
        case StateType::SELF_CHECK: return "SELF_CHECK";
        case StateType::READY: return "READY";
        case StateType::OPERATIONAL: return "OPERATIONAL";
        case StateType::MANUAL: return "MANUAL";
        case StateType::PAUSED: return "PAUSED";
        case StateType::ERROR: return "ERROR";
        case StateType::TELEOP: return "TELEOP";
        case StateType::AUTONOMOUS: return "AUTONOMOUS";
        case StateType::SEMI_AUTONOMOUS: return "SEMI_AUTONOMOUS";
        case StateType::RECOVERING: return "RECOVERING";
        case StateType::EMERGENCY_STOP: return "EMERGENCY_STOP";
        case StateType::SHUTDOWN: return "SHUTDOWN";
        case StateType::CALIBRATING: return "CALIBRATING";
        case StateType::DIAGNOSTICS: return "DIAGNOSTICS";
        case StateType::SAFE_MODE: return "SAFE_MODE";
        case StateType::DEGRADED: return "DEGRADED";
        case StateType::CLEANING: return "CLEANING";
        case StateType::NAVIGATING: return "NAVIGATING";
        case StateType::TASK_EXECUTION: return "TASK_EXECUTION";
        case StateType::IDLE: return "IDLE";
        case StateType::STANDBY: return "STANDBY";
        case StateType::MISSION_PLANNING: return "MISSION_PLANNING";
        case StateType::AVOID_OBSTACLE: return "AVOID_OBSTACLE";
        case StateType::TEST_MODE: return "TEST_MODE";
        case StateType::MAINTENANCE: return "MAINTENANCE";
        case StateType::SAFETY_HOLD: return "SAFETY_HOLD";
        default: return "UNKNOWN(" + std::to_string(static_cast<uint32_t>(type)) + ")";
    }
}

} // namespace ns_fsm
