#ifndef HFSM_ERRORCODE_HPP
#define HFSM_ERRORCODE_HPP

namespace ns_fsm {

/**
 * @enum ErrorCode
 * @brief Códigos de error específicos del HFSM
 */
/**
 * @enum ErrorCode
 * @brief Códigos de error específicos del HFSM
 */
enum class ErrorCode : int32_t {
    SUCCESS = 0,
    TRANSITION_ERROR = 100,

    // Errores relacionados con estados
    STATE_NOT_FOUND = 1000,
    STATE_TRANSITION_ERROR = 1001,
    STATE_UPDATE_ERROR = 1002,
    STATE_REGISTRATION_ERROR = 1003,
    
    // Errores relacionados con módulos
    MODULE_SHUTDOWN_ERROR = 1100,
    MODULE_REGISTRATION_FAILED = 1101,
    MODULE_INITIALIZATION_FAILED = 1102,
    
    // Errores relacionados con apagado
    SHUTDOWN_ERROR = 1200,
    GRACEFUL_SHUTDOWN_TIMEOUT = 1201,
    CLEANING_FAILED = 1202,
    SHUTDOWN_IN_PROGRESS = 1203,
    
    // Errores generales del sistema
    EVENT_PROCESSING_ERROR = 1300,
    WATCHDOG_TIMEOUT = 1301,
    THREAD_START_FAILED = 1302,
    INVALID_CONFIGURATION = 1303,
    ALREADY_INITIALIZED = 1304,
    NOT_INITIALIZED = 1305,
    DEADLOCK_DETECTED = 1306,
    INVALID_PARAMETER = 1307,
    TIMEOUT = 1308,
    INVALID_OPERATION = 1309,
    
    // Error crítico
    CRITICAL_FAILURE = 9999
};

} // namespace ns_fsm

#endif  // HFSM_ERRORCODE_HPP
