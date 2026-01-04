// hfsm_event_types.hpp - TIPOS DE EVENTOS Y ESTRUCTURAS BÁSICAS
#ifndef HFSM_EVENT_TYPES_HPP
#define HFSM_EVENT_TYPES_HPP

#include <cstdint>
#include <string>
#include <variant>
#include <memory>
#include <chrono>
#include <optional>
#include <nlohmann/json.hpp>

namespace ns_fsm {

// ============================================================================
// TIPOS ENUMERADOS BÁSICOS
// ============================================================================

// Tipos de eventos del sistema
enum class EventType : uint32_t {
    // Eventos de control del sistema (0-49)
    SYSTEM_START = 0,
    SYSTEM_SHUTDOWN,
    SYSTEM_ERROR,
    SYSTEM_RECOVERY,
    SYSTEM_READY,
    SYSTEM_CHECK_RESULT,
    SHUTDOWN_REQUEST = 7,
    SHUTDOWN_INITIATED,
    SHUTDOWN_IMMEDIATE,
    SHUTDOWN_PREPARE,
    
    // Eventos de control básico (15-29)
    PAUSE = 15,
    RESUME,
    
    // Eventos de estados HFSM (30-49)
    STATE_CHANGED = 30,
    STATE_COMPLETED,
    STATE_FAILED,
    STATE_TRANSITION_REQUESTED,
    STATE_OPERATIONAL_CHANGED,
    
    // Eventos de red (50-99)
    NETWORK_COMMAND = 50,
    NETWORK_TELEOP = 56,
    NETWORK_DISCONNECTED = 57,
    
    // Eventos de respuesta HFSM→Network (100-149)
    NETWORK_STATE_UPDATE = 100,
    NETWORK_TASK_RESULT,
    NETWORK_ERROR,
    TELEOP_REQUEST_RECEIVED,
    AUTONOMOUS_REQUEST_RECEIVED,
    
    // Eventos de módulos (150-199)
    MODULE_INITIALIZED = 150,
    MODULE_SHUTDOWN,
    MODULE_ERROR,
    
    // Eventos de seguridad (200-249)
    SECURITY_AUTH_REQUEST = 200,
    SECURITY_AUTH_SUCCESS,
    SECURITY_AUTH_FAILED,

    COMPONENT_FAILURE = 210,      // ✅ GENÉRICO (reemplaza SENSOR_FAILURE_DETECTED)
    COMPONENT_RECOVERED = 211,    // ✅ GENÉRICO (reemplaza SENSOR_RECOVERED)
    SYSTEM_DEGRADED = 212,        // ✅ GENÉRICO (reemplaza SENSOR_DEGRADED_MODE)
    SYSTEM_FULLY_OPERATIONAL = 213,

    // Eventos de hardware (250-299)
    HARDWARE_FAULT = 253,
    
    // Eventos de batería (300-349)
    LOW_BATTERY = 301,
    CRITICAL_BATTERY = 303,
    
    // Eventos de monitoreo de sistema (350-399)
    HIGH_CPU_USAGE = 350,
    HIGH_TEMPERATURE,
    
    // Eventos de modo de operación (400-449)
    MODE_AUTONOMOUS = 400,
    MODE_TELEOP,
    
    // Eventos de diagnóstico (450-469)
    DIAGNOSTICS_START = 450,
    DIAGNOSTICS_COMPLETE,
    DIAGNOSTICS_FAILED,
    
    // Eventos de emergencia (470-489)
    EMERGENCY_STOP = 470,
    EMERGENCY_BUTTON_PRESSED,
    EMERGENCY_CLEARED,
    
    // Eventos de configuración (490-509)
    CONFIG_LOADED = 490,
    CONFIG_CHANGED,
    CONFIG_ERROR,
    
    // Eventos de logging (510-529)
    LOG_CRITICAL = 510,
    LOG_ERROR,
    LOG_WARNING,
    LOG_INFO,
    LOG_DEBUG,
    
    // Eventos de error genéricos (530-549)
    CRITICAL_ERROR = 530,
    TEMPORARY_FAILURE,
    COMMUNICATION_ERROR,
    
    // Eventos de velocidad (550-559)
    SPEED_LIMIT_CHANGED = 550,
    
    // Eventos de auditoría (560-579)
    AUDIT_EVENT = 560,
    AUDIT_SECURITY_VIOLATION,
    
    // Eventos de performance (580-599)
    PERFORMANCE_METRIC = 580,
    PERFORMANCE_ALERT,
    
    // Eventos de limpieza (600-609)
    CLEANUP_STARTED = 600,
    CLEANUP_COMPLETE,
    CLEANUP_FAILED,
    
    // Eventos de comandos específicos (610-619)
    SHUTDOWN_COMMAND = 610,
    SHUTDOWN_TIMEOUT_WARNING,

    // Eventos de navegación (700-749)
    NAV_GOAL_RECEIVED = 700,
    NAV_GOAL_REACHED,
    NAV_OBSTACLE_DETECTED,
    NAV_PATH_BLOCKED,
    NAV_EMERGENCY_STOP,
    NAV_LOCALIZATION_LOST,
    NAV_LOCALIZATION_RECOVERED,
    
    // Eventos de SLAM (750-779)
    LOCALIZATION_LOW_CONFIDENCE = 750,
    LOCALIZATION_SUCCESS,
    LOCALIZATION_FAILED,
    
    // Eventos de tareas (780-819)
    TASK_START = 780,
    TASK_COMPLETE,
    TASK_FAILED,
    
    // Eventos de recuperación (820-839)
    RECOVERY_SUCCESS = 820,
    RECOVERY_FAILED,
    RECOVERY_REQUEST,
    
    // Eventos de watchdog (840-859)
    WATCHDOG_TIMEOUT = 840,
    
    // Eventos de timer (860-869)
    TIMER_TICK = 860,
    
    // Eventos de teleoperación (870-889)
    TELEOP_STATE_ENTER = 870,
    TELEOP_STATE_EXIT,
    TELEOP_COMMAND,
    RETURN_TO_IDLE,
    
    // Eventos de estados autónomos (890-899)
    AUTONOMOUS_STATE_ENTER = 890,
    AUTONOMOUS_STATE_EXIT,
    
    // Eventos de misiones (900-949)
    MISSION_STARTED = 900,
    MISSION_RECEIVED,
    MISSION_COMPLETE,
    MISSION_ABORT,
    MAP_SAVE_REQUEST,
    EXPLORATION_COMPLETE,
    MANUAL_RESET,
    WAYPOINT_REACHED,
    
    // Eventos de salud del sistema (950-969)
    HEARTBEAT = 950,
    KEEPALIVE,
    SYSTEM_HEALTH_STATUS,
    
    // Eventos reservados/user defined (1000-1999)
    USER_DEFINED_START = 1000,

    // Eventos avanzados de robótica (1050-1099)
    ROBOT_NAVIGATION_START = 1050,
    ROBOT_NAVIGATION_COMPLETE,
    ROBOT_NAVIGATION_FAILED,
    ROBOT_PATH_PLANNING_START,
    ROBOT_PATH_PLANNING_COMPLETE,
    ROBOT_COLLISION_DETECTED,
    ROBOT_OBSTACLE_AVOIDANCE_START,
    ROBOT_MAPPING_START,
    ROBOT_MAPPING_COMPLETE,
    ROBOT_LOCALIZATION_UPDATED,
    ROBOT_SLAM_ACTIVE,
    ROBOT_TELEOP_OVERRIDE,
    ROBOT_AUTONOMOUS_OVERRIDE,
    ROBOT_MANIPULATION_START,
    ROBOT_MANIPULATION_COMPLETE,
    ROBOT_GRIPPER_OPEN,
    ROBOT_GRIPPER_CLOSE,
    ROBOT_ARM_MOVE_TO,
    ROBOT_ARM_HOME,
    ROBOT_VISION_OBJECT_DETECTED,
    ROBOT_VISION_TRACKING_START,
    ROBOT_VISION_TRACKING_LOST,
    ROBOT_BATTERY_SWAP_REQUEST,
    ROBOT_CHARGING_START,
    ROBOT_CHARGING_COMPLETE,
    ROBOT_FLEET_COORDINATION,
    ROBOT_TASK_DELEGATION,
    ROBOT_FORMATION_START,
    ROBOT_FORMATION_MAINTAIN,
    ROBOT_FORMATION_BREAK,
    ROBOT_LEARNING_START,
    ROBOT_LEARNING_COMPLETE,
    ROBOT_ADAPTIVE_BEHAVIOR,
    ROBOT_PREDICTIVE_MAINTENANCE,
    
    // Evento desconocido
    UNKNOWN = 9999
};

// Tipos de estado para la máquina de estados
enum class StateType : uint32_t {
    UNINITIALIZED = 0,
    INITIALIZING,
    SELF_CHECK,
    READY,
    OPERATIONAL,
    MANUAL,
    PAUSED,
    ERROR,
    TELEOP,
    AUTONOMOUS,
    SEMI_AUTONOMOUS,
    RECOVERING,
    EMERGENCY_STOP,
    SHUTDOWN,
    CALIBRATING,
    DIAGNOSTICS,
    SAFE_MODE,
    DEGRADED,
    CLEANING,
    NAVIGATING,
    TASK_EXECUTION,
    IDLE,
    STANDBY,
    MISSION_PLANNING,
    AVOID_OBSTACLE,
    TEST_MODE,
    MAINTENANCE,
    SAFETY_HOLD,

    // Estados avanzados de robótica (100-199)
    NAVIGATING_TO_GOAL = 100,
    PATH_PLANNING = 101,
    OBSTACLE_AVOIDANCE = 102,
    MAPPING = 103,
    SLAM = 104,
    TELEOPERATION = 105,  // Ya existe TELEOP, pero añadimos alias
    MANIPULATING = 106,
    GRASPING = 107,
    PLACING = 108,
    VISION_PROCESSING = 109,
    OBJECT_TRACKING = 110,
    CHARGING = 111,
    BATTERY_SWAP = 112,
    FLEET_COORDINATION = 113,
    FORMATION_CONTROL = 114,
    TASK_COORDINATION = 115,
    LEARNING = 116,
    ADAPTING = 117,
    PREDICTIVE_MAINTENANCE = 118,
    RECONFIGURING = 119,
    CALIBRATING_SENSORS = 120,  // Más específico que CALIBRATING

    UNKNOWN = 0xFFFFFFFF
};

// ============================================================================
// ESTRUCTURAS DE DATOS BÁSICAS
// ============================================================================

// Información de error
struct ErrorInfo {
    uint32_t error_code{0};
    std::string module;
    std::string subsystem;
    std::string description;
    std::string recommendation;
    std::chrono::system_clock::time_point timestamp;
    nlohmann::json context;
    int severity{0};
    
    std::string toString() const {
        return "[" + std::to_string(error_code) + "] " + module + 
               ": " + description + " (severity: " + std::to_string(severity) + ")";
    }
};

// Límites de velocidad
struct SpeedLimits {
    float max_linear{0.5f};
    float max_angular{1.0f};
    
    SpeedLimits() = default;
    SpeedLimits(float lin, float ang) : max_linear(lin), max_angular(ang) {}
};

// Variante de datos de evento
using EventData = std::variant<
    std::monostate,
    ErrorInfo,
    SpeedLimits,
    std::string,
    int32_t,
    uint32_t,
    double,
    float,
    bool,
    nlohmann::json,
    std::vector<uint8_t>
>;

// ============================================================================
// ESTRUCTURA DE EVENTO PRINCIPAL
// ============================================================================

struct Event {
    EventType type;
    EventData data;
    std::chrono::system_clock::time_point timestamp;
    std::string source_module;
    std::string source_component;
    uint32_t sequence_number{0};
    std::string correlation_id;
    int priority{0};
    bool requires_acknowledgment{false};
    nlohmann::json metadata;
    
    // Constructores
    Event();
    Event(EventType type, const std::string& source = "system");
    Event(EventType type, EventData data, const std::string& source = "system");
    Event(EventType type, const char* msg, const std::string& source = "system");
    Event(EventType type, int value, const std::string& source = "system");
    Event(EventType type, double value, const std::string& source = "system");
    Event(EventType type, bool value, const std::string& source = "system");
    
    // Métodos
    bool hasData() const;
    
    template<typename T> 
    std::optional<T> getDataAs() const {
        if (std::holds_alternative<T>(data)) {
            return std::get<T>(data);
        }
        return std::nullopt;
    }
    
    bool isError() const;
    bool isHighPriority() const;
    std::string toString() const;
    nlohmann::json toJson() const;
};

// ============================================================================
// FUNCIONES DE UTILIDAD
// ============================================================================

std::string eventTypeToString(EventType type);
std::string stateTypeToString(StateType type);

} // namespace ns_fsm

#endif // HFSM_EVENT_TYPES_HPP
