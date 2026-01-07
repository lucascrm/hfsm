/**
 * @file     hfsm_config_core.hpp
 * @brief    Definición centralizada de estructura de configuración del HFSM
 * @author   Lucas C.
 * @date     27/12/2025
 * @version  1.0
 * @license  MIT
 * 
 * @details  Este archivo define la estructura HFSMConfig que centraliza todos los
 *           parámetros de configuración del sistema HFSM, incluyendo:
 *           - Timeouts de estados y eventos
 *           - Parámetros de control del loop principal
 *           - Configuración de watchdog y logging
 *           - Gestión de errores y shutdown
 *           - Estructuras organizadas para mejor mantenibilidad
 * 
 * @note     Usa una estructura organizada interna (Timeouts) para agrupar
 *           parámetros relacionados, manteniendo campos legacy para compatibilidad.
 *           El método syncAll() mantiene consistencia entre campos.
 * 
 * @warning  Los campos legacy están marcados como obsoletos. Usar la estructura
 *           organizada para nuevas funcionalidades.
 * 
 * @see      hfsm_config_manager.hpp - Sistema de gestión de configuración completa
 */
#ifndef HFSM_CONFIG_CORE_HPP
#define HFSM_CONFIG_CORE_HPP

#include <cstdint>
#include <chrono>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>

namespace ns_fsm {

// ============================================================================
// ESTRUCTURA DE CONFIGURACIÓN ÚNICA
// ============================================================================

struct HFSMConfig {
    // Estructura interna para timeouts
    struct Timeouts {
        uint32_t state_timeout_ms{5000};      // Timeout por estado
        uint32_t global_timeout_ms{30000};    // Timeout global
        uint32_t event_processing_timeout_ms{100}; // Timeout para eventos
        
        Timeouts() = default;
        
        Timeouts(uint32_t state, uint32_t global, uint32_t event)
            : state_timeout_ms(state)
            , global_timeout_ms(global)
            , event_processing_timeout_ms(event) {}
    };
    
    // Campos de control
    uint32_t loop_sleep_ms{10};               // Sleep entre iteraciones
    uint32_t max_consecutive_errors{10};      // Máximo errores consecutivos
    uint32_t shutdown_timeout_ms{3000};       // Timeout para shutdown
    bool enable_watchdog{true};               // Habilitar watchdog
    bool log_transitions{true};               // Log de transiciones
    bool log_events{false};                   // Log detallado de eventos
    
    // Timeouts organizados
    Timeouts timeouts;
    
    // Campos legacy para compatibilidad
    uint32_t state_timeout_ms{5000};          // Alias para timeouts.state_timeout_ms
    uint32_t global_timeout_ms{30000};        // Alias para timeouts.global_timeout_ms
    uint32_t event_processing_timeout_ms{100}; // Alias para timeouts.event_processing_timeout_ms
    uint32_t watchdog_timeout_ms{5000};       // Legacy: igual a state_timeout_ms
    
    // Constructor por defecto
    HFSMConfig() {
        syncAll();
    }
    
    // Constructor con parámetros
    HFSMConfig(uint32_t loop_ms, uint32_t max_errors, uint32_t shutdown_ms,
               bool watchdog, bool log_trans, bool log_ev)
        : loop_sleep_ms(loop_ms)
        , max_consecutive_errors(max_errors)
        , shutdown_timeout_ms(shutdown_ms)
        , enable_watchdog(watchdog)
        , log_transitions(log_trans)
        , log_events(log_ev) {
        syncAll();
    }
    
    // Método para sincronizar todos los valores
    void syncAll() {
        // 1. Sincronizar timeouts organizados con campos individuales
        timeouts.state_timeout_ms = state_timeout_ms;
        timeouts.global_timeout_ms = global_timeout_ms;
        timeouts.event_processing_timeout_ms = event_processing_timeout_ms;
        
        // 2. Sincronizar watchdog legacy
        watchdog_timeout_ms = enable_watchdog ? state_timeout_ms : 0;
    }
    
    // Métodos de acceso con nombres claros
    uint32_t getLoopSleepMs() const { return loop_sleep_ms; }
    uint32_t getMaxConsecutiveErrors() const { return max_consecutive_errors; }
    uint32_t getShutdownTimeoutMs() const { return shutdown_timeout_ms; }
    bool getEnableWatchdog() const { return enable_watchdog; }
    bool getLogTransitions() const { return log_transitions; }
    bool getLogEvents() const { return log_events; }
    
    uint32_t getStateTimeoutMs() const { return timeouts.state_timeout_ms; }
    uint32_t getGlobalTimeoutMs() const { return timeouts.global_timeout_ms; }
    uint32_t getEventProcessingTimeoutMs() const { return timeouts.event_processing_timeout_ms; }
    
    // Métodos para gestión de sensores
//    const SensorManagement& getSensorManagement() const { return sensor_mgmt; }
//    SensorManagement& getSensorManagement() { return sensor_mgmt; }

    // Método estático para configuración por defecto
    static HFSMConfig Default() {
        HFSMConfig config;
        
        config.syncAll();
        return config;
    }
    
};

} // namespace ns_fsm

#endif // HFSM_CONFIG_CORE_HPP
