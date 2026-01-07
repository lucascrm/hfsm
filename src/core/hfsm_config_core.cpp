/**
 * @file     hfsm_config_core.cpp
 * @brief    Implementación de estructuras básicas de configuración del HFSM
 * @author   Lucas C.
 * @date     27/12/2025
 * @version  1.0
 * @license  MIT
 * 
 * @details  Este archivo contiene la implementación de los métodos de HFSMConfig,
 *           incluyendo constructores, métodos de sincronización y configuración
 *           por defecto. Proporciona:
 *           - Implementación de sincronización entre campos legacy y organizados
 *           - Constructor por defecto con valores robustos
 *           - Gestión coherente de timeouts y parámetros de control
 * 
 * @note     La sincronización (syncAll) es crítica para mantener consistencia
 *           entre campos legacy y la nueva estructura organizada de timeouts.
 * 
 * @warning  Cambiar manualmente los campos legacy sin llamar a syncAll() puede
 *           causar inconsistencias en el sistema.
 * 
 * @see      hfsm_config_core.hpp - Declaración de estructuras de configuración
 */
#include "hfsm_config_core.hpp"

namespace ns_fsm {

// ============================================================================
// MÉTODOS DE HFSMCONFIG
// ============================================================================

// Constructor con todos los parámetros
HFSMConfig::HFSMConfig(uint32_t state_ms, uint32_t global_ms, uint32_t event_ms,
                      uint32_t loop_ms, uint32_t max_errors, uint32_t shutdown_ms,
                      bool watchdog, bool log_trans, bool log_ev)
    : loop_sleep_ms(loop_ms)
    , max_consecutive_errors(max_errors)
    , shutdown_timeout_ms(shutdown_ms)
    , enable_watchdog(watchdog)
    , log_transitions(log_trans)
    , log_events(log_ev) {
    
    timeouts.state_timeout_ms = state_ms;
    timeouts.global_timeout_ms = global_ms;
    timeouts.event_processing_timeout_ms = event_ms;
    
    syncAll();
}

// Método estático para configuración por defecto
HFSMConfig HFSMConfig::Default() {
    HFSMConfig config;
    
    // Valores por defecto robustos
    config.loop_sleep_ms = 10;
    config.max_consecutive_errors = 10;
    config.shutdown_timeout_ms = 3000;
    config.enable_watchdog = true;
    config.log_transitions = true;
    config.log_events = false;
    
    config.timeouts.state_timeout_ms = 5000;
    config.timeouts.global_timeout_ms = 30000;
    config.timeouts.event_processing_timeout_ms = 100;
    
    config.syncAll();
    return config;
}

} // namespace ns_fsm
