// hfsm_core.cpp - IMPLEMENTACIÓN COMPLETA DEL HFSM CORE CON LOGGING INTEGRADO
#include "hfsm_core.hpp"
#include "hfsm_event_types.hpp"
#include "modules/manager/ModuleManager.h"
#include "hardware/utils/Diagnostics.h"
#include "hfsm_logging.hpp"  // Incluir el header del logger

#include <iostream>
#include <sstream>
#include <iomanip>

namespace ns_fsm {

// ============================================================================
// FUNCIONES DE LOGGING LOCALES (para evitar conflictos de macros)
// ============================================================================

namespace logging {
    inline void log_debug(const std::string& subsystem, 
                         const std::string& message,
                         const nlohmann::json& context = {}) {
        HFSMLogger::getInstance().logStructured(
            subsystem,
            hardware::DiagnosticLevel::DEBUG,
            message,
            context
        );
    }
    
    inline void log_info(const std::string& subsystem,
                        const std::string& message,
                        const nlohmann::json& context = {}) {
        HFSMLogger::getInstance().logStructured(
            subsystem,
            hardware::DiagnosticLevel::INFO,
            message,
            context
        );
    }
    
    inline void log_warn(const std::string& subsystem,
                        const std::string& message,
                        const nlohmann::json& context = {}) {
        HFSMLogger::getInstance().logStructured(
            subsystem,
            hardware::DiagnosticLevel::WARN,
            message,
            context
        );
    }
    
    inline void log_error(const std::string& subsystem,
                         const std::string& message,
                         const nlohmann::json& context = {}) {
        HFSMLogger::getInstance().logStructured(
            subsystem,
            hardware::DiagnosticLevel::ERROR,
            message,
            context
        );
    }
    
    inline void log_fatal(const std::string& subsystem,
                         const std::string& message,
                         const nlohmann::json& context = {}) {
        HFSMLogger::getInstance().logStructured(
            subsystem,
            hardware::DiagnosticLevel::FATAL,
            message,
            context
        );
    }
} // namespace logging

// ============================================================================
// IMPLEMENTACIÓN DE EVENT BUS
// ============================================================================

void EventBus::subscribe(const std::string& id, uint32_t event_type, 
                         std::function<void(const Event&)> callback, int priority) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    Subscription sub;
    sub.id = id;
    sub.callback = std::move(callback);
    sub.priority = priority;
    
    auto& subs = subscribers_[event_type];
    subs.push_back(std::move(sub));
    
    // Ordenar por prioridad (descendente)
    std::sort(subs.begin(), subs.end(),
        [](const Subscription& a, const Subscription& b) {
            return a.priority > b.priority;
        });
    
    logging::log_debug("EventBus", 
        std::string("Suscripción creada: ") + id + 
        " para evento: " + std::to_string(event_type),
        {{"priority", priority}});
}

void EventBus::unsubscribe(const std::string& id, uint32_t event_type) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    auto it = subscribers_.find(event_type);
    if (it != subscribers_.end()) {
        auto& subs = it->second;
        size_t old_size = subs.size();
        subs.erase(
            std::remove_if(subs.begin(), subs.end(),
                [&](const Subscription& sub) {
                    return sub.id == id;
                }),
            subs.end()
        );
        
        if (subs.size() != old_size) {
            logging::log_debug("EventBus", 
                std::string("Suscripción eliminada: ") + id + 
                " para evento: " + std::to_string(event_type));
        }
    }
}

void EventBus::publish(const Event& event) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        event_queue_.push(event);
    }
    
    logging::log_debug("EventBus", 
        std::string("Evento publicado: ") + eventTypeToString(event.type),
        {
            {"source", event.source_module},
            {"sequence", event.sequence_number},
            {"has_data", event.hasData()}
        });
    
    cv_.notify_one();
}

bool EventBus::processOne() {
    // Extraer evento
    Event event;
    {
        std::unique_lock<std::mutex> lock(mutex_);
        
        if (!cv_.wait_for(lock, std::chrono::milliseconds(1),
            [this] { return !event_queue_.empty(); })) {
            return false;
        }
        
        event = event_queue_.front();
        event_queue_.pop();
    }

    std::vector<Subscription> subs_specific;
    std::vector<Subscription> subs_all;
    
    {
        std::lock_guard<std::mutex> lock(mutex_);
    
        // 1. Suscriptores específicos para este tipo de evento
        auto it_specific = subscribers_.find(static_cast<uint32_t>(event.type));
        if (it_specific != subscribers_.end()) {
            subs_specific = it_specific->second;
        }
    
        // 2. Suscriptores a TODOS los eventos (EventType::UNKNOWN)
        auto it_all = subscribers_.find(static_cast<uint32_t>(EventType::UNKNOWN));
        if (it_all != subscribers_.end()) {
            subs_all = it_all->second;
        }
    }

    logging::log_debug("EventBus", 
        std::string("Procesando evento: ") + eventTypeToString(event.type),
        {
            {"specific_subscribers", subs_specific.size()},
            {"general_subscribers", subs_all.size()}
        });

    // Ejecutar primero los específicos, luego los generales
    for (const auto& sub : subs_specific) {
        try {
            sub.callback(event);
            logging::log_debug("EventBus", 
                std::string("Callback ejecutado: ") + sub.id,
                {{"priority", sub.priority}});
        } catch (const std::exception& e) {
            logging::log_error("EventBus", 
                std::string("Error en suscriptor específico '") + sub.id + "': " + e.what(),
                {{"error_code", static_cast<int>(ErrorCode::EVENT_PROCESSING_ERROR)}});
        }
    }

    for (const auto& sub : subs_all) {
        try {
            sub.callback(event);
            logging::log_debug("EventBus", 
                std::string("Callback general ejecutado: ") + sub.id,
                {{"priority", sub.priority}});
        } catch (const std::exception& e) {
            logging::log_error("EventBus", 
                std::string("Error en suscriptor general '") + sub.id + "': " + e.what(),
                {{"error_code", static_cast<int>(ErrorCode::EVENT_PROCESSING_ERROR)}});
        }
    }

    return true;
}

EventBus::Stats EventBus::getStats() const {
    std::lock_guard<std::mutex> lock(mutex_);
    Stats stats;
    stats.pending_events = event_queue_.size();
    stats.event_types = subscribers_.size();
    
    for (const auto& pair : subscribers_) {
        stats.total_subscribers += pair.second.size();
    }
    
    return stats;
}

void EventBus::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    size_t cleared = event_queue_.size();
    std::queue<Event> empty;
    std::swap(event_queue_, empty);
    
    logging::log_warn("EventBus", 
        std::string("Cola de eventos limpiada: ") + std::to_string(cleared) + " eventos eliminados");
}

// ============================================================================
// IMPLEMENTACIÓN DE HFSMCore
// ============================================================================

// Constructor
HFSMCore::HFSMCore(ModuleManager* module_manager, const HFSMConfig& config)
    : module_manager_(module_manager)
    , config_(config)
    , last_state_change_(std::chrono::steady_clock::now())
    , last_update_(std::chrono::steady_clock::now()) {
    
    config_.syncAll();
    
    logging::log_debug("HFSM", 
        "HFSMCore creado",
        {
            {"module_manager", module_manager_ != nullptr},
            {"state_timeout", config_.getStateTimeoutMs()},
            {"watchdog_enabled", config_.getEnableWatchdog()}
        });
}

// Destructor
HFSMCore::~HFSMCore() {
    logging::log_debug("HFSM", "Destructor HFSMCore llamado");
    shutdown(true);
}

// Métodos privados
void HFSMCore::runLoop() {
    logging::log_info("HFSM", "Hilo principal iniciado");
    
    auto last_stats_log = std::chrono::steady_clock::now();
    int consecutive_errors = 0;
    
    while (running_.load()) {
        auto loop_start = std::chrono::steady_clock::now();
        stats_.total_loops++;
        
        try {
            // 1. Procesar eventos (máximo 50 por iteración)
            int events_processed = 0;
            for (int i = 0; i < 50 && event_bus_.processOne(); i++) {
                events_processed++;
                stats_.events_processed++;
            }
            
            // 2. Ejecutar update del estado actual
            bool state_updated = false;
            {
                std::unique_lock<std::recursive_mutex> lock(mutex_, std::try_to_lock);
                if (lock.owns_lock() && current_state_ && running_.load()) {
                    try {
                        logging::log_debug("HFSM", 
                            std::string("Actualizando estado: ") + current_state_->getName());
                        
                        current_state_->update();
                        state_updated = true;
                        consecutive_errors = 0;
                        
                        // Verificar timeout
                        checkStateTimeout();
                        
                    } catch (const std::exception& e) {
                        logging::log_error("HFSM", 
                            std::string("Error de actualización de estado: ") + e.what(),
                            {
                                {"error_code", static_cast<int>(ErrorCode::STATE_UPDATE_ERROR)},
                                {"state", current_state_->getName()}
                            });
                        consecutive_errors++;
                        stats_.errors++;
                    }
                }
            }
            
            // 3. Ejecutar updates de módulos
            if (state_updated && !shutdown_requested_.load() && module_manager_) {
                try {
                    auto now = std::chrono::steady_clock::now();
                    auto delta_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        now - last_update_).count();
                    last_update_ = now;
                    
                    logging::log_debug("HFSM", 
                        "Actualizando módulos",
                        {{"delta_ms", delta_ms}});
                    
                    module_manager_->updateAll(static_cast<double>(delta_ms) / 1000.0);
                    
                } catch (const std::exception& e) {
                    logging::log_error("HFSM", 
                        std::string("Error de actualización del módulo: ") + e.what(),
                        {{"error_code", static_cast<int>(ErrorCode::MODULE_INITIALIZATION_FAILED)}});
                    consecutive_errors++;
                    stats_.errors++;
                }
            }
            
            // 4. Manejar demasiados errores consecutivos
            if (consecutive_errors >= config_.getMaxConsecutiveErrors()) {
                logging::log_fatal("HFSM", 
                    "Demasiados errores consecutivos, apagado de emergencia",
                    {
                        {"error_code", static_cast<int>(ErrorCode::CRITICAL_FAILURE)},
                        {"consecutive_errors", consecutive_errors},
                        {"max_allowed", config_.getMaxConsecutiveErrors()}
                    });
                
                emergencyShutdown();
                break;
            }
            
            // 5. Log de estadísticas periódico
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::seconds>(
                now - last_stats_log).count() >= 10) {
                
                logStatistics();
                last_stats_log = now;
            }
            
        } catch (const std::exception& e) {
            logging::log_fatal("HFSM", 
                std::string("Excepción no controlada en el bucle principal: ") + e.what(),
                {{"error_code", static_cast<int>(ErrorCode::CRITICAL_FAILURE)}});
            
            consecutive_errors++;
            stats_.errors++;
            
            if (consecutive_errors >= config_.getMaxConsecutiveErrors()) {
                emergencyShutdown();
                break;
            }
        }
        
        // 6. Control de frecuencia
        auto loop_end = std::chrono::steady_clock::now();
        auto loop_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            loop_end - loop_start);
        
        if (loop_duration < std::chrono::milliseconds(config_.getLoopSleepMs())) {
            auto sleep_time = std::chrono::milliseconds(config_.getLoopSleepMs()) - loop_duration;
            logging::log_debug("HFSM", 
                "Controlando frecuencia de loop",
                {
                    {"loop_duration_ms", loop_duration.count()},
                    {"sleep_time_ms", sleep_time.count()},
                    {"target_sleep_ms", config_.getLoopSleepMs()}
                });
            
            std::this_thread::sleep_for(sleep_time);
        }
    }
    
    logging::log_info("HFSM", "El hilo principal se detuvo");
}

void HFSMCore::checkStateTimeout() {
    if (!config_.getEnableWatchdog() || !current_state_) return;
    
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - last_state_change_);
    
    config_.syncAll();
    
    auto timeout_ms = config_.getEnableWatchdog() ? 
        config_.getStateTimeoutMs() : 0;
    
    if (timeout_ms > 0 && elapsed.count() > timeout_ms) {
        logging::log_warn("HFSM", 
            std::string("Se detectó tiempo de espera de estado: ") + current_state_->getName(),
            {
                {"elapsed_ms", elapsed.count()},
                {"timeout_ms", timeout_ms},
                {"state", current_state_->getName()}
            });
        
        // Publicar evento de timeout
        ErrorInfo timeout_info;
        timeout_info.error_code = static_cast<uint32_t>(ErrorCode::WATCHDOG_TIMEOUT);
        timeout_info.description = "State timeout: " + current_state_->getName();
        timeout_info.severity = 2;
        timeout_info.module = "HFSM";
        
        Event event(EventType::WATCHDOG_TIMEOUT, timeout_info, "HFSM");
        event_bus_.publish(event);
    }
}

void HFSMCore::emergencyShutdown() {
    logging::log_error("HFSM", "PARADA DE EMERGENCIA INICIADA",
        {{"error_code", static_cast<int>(ErrorCode::CRITICAL_FAILURE)}});

    running_ = false;
    shutdown_requested_ = true;

    // Forzar transición a SHUTDOWN si existe
    auto it = states_.find(static_cast<uint32_t>(StateType::SHUTDOWN));
    if (it != states_.end() && current_state_ != it->second.get()) {
        try {
            logging::log_warn("HFSM", "Forzando transición a SHUTDOWN");
            transitionToInternal(static_cast<uint32_t>(StateType::SHUTDOWN), true);
        } catch (...) {
            logging::log_error("HFSM", "Error durante transición de emergencia");
        }
    }

    // Apagar módulos
    if (module_manager_) {
        logging::log_info("HFSM", "Apagando módulos de emergencia");
        try {
            module_manager_->shutdownAll(true);
            logging::log_info("HFSM", "Módulos apagados");
        } catch (const std::exception& e) {
            logging::log_error("HFSM", 
                std::string("Error apagando módulos: ") + e.what());
        }
    }

    event_bus_.clear();
    logging::log_info("HFSM", "Apagado de emergencia completo");
}

void HFSMCore::logStatistics() {
    auto bus_stats = event_bus_.getStats();
    
    logging::log_info("HFSM", "Estadísticas del sistema",
        {
            {"total_loops", stats_.total_loops},
            {"events_processed", stats_.events_processed},
            {"state_transitions", stats_.state_transitions},
            {"errors", stats_.errors},
            {"pending_events", bus_stats.pending_events},
            {"event_types", bus_stats.event_types},
            {"total_subscribers", bus_stats.total_subscribers},
            {"running", isRunning()},
            {"initialized", isInitialized()}
        });
}

ErrorCode HFSMCore::transitionToInternal(uint32_t target, bool emergency) {
    // Validar que el estado existe
    auto it = states_.find(target);
    if (it == states_.end()) {
        logging::log_error("HFSM", 
            std::string("Estado no encontrado: ") + std::to_string(target),
            {
                {"error_code", static_cast<int>(ErrorCode::STATE_NOT_FOUND)},
                {"target_state", target}
            });
        return ErrorCode::STATE_NOT_FOUND;
    }
    
    // Verificar si ya estamos en este estado
    if (current_state_ && current_state_->getType() == target) {
        logging::log_debug("HFSM", 
            std::string("Ya en estado: ") + current_state_->getName());
        return ErrorCode::SUCCESS;
    }
    
    // Validar transición (excepto en emergencia)
    if (!emergency && current_state_) {
        if (!current_state_->canTransitionTo(target)) {
            logging::log_error("HFSM", 
                std::string("Transición no válida: ") + current_state_->getName() +
                " -> " + std::to_string(target),
                {
                    {"error_code", static_cast<int>(ErrorCode::STATE_TRANSITION_ERROR)},
                    {"from_state", current_state_->getName()},
                    {"to_state", it->second->getName()}
                });
            return ErrorCode::STATE_TRANSITION_ERROR;
        }
        
        if (current_state_->isFinalState()) {
            logging::log_error("HFSM", 
                std::string("No se puede pasar del estado final: ") + 
                current_state_->getName(),
                {{"error_code", static_cast<int>(ErrorCode::STATE_TRANSITION_ERROR)}});
            return ErrorCode::STATE_TRANSITION_ERROR;
        }
    }
    
    // Log de transición usando la función del logger
    HFSMLogger::getInstance().logStateTransition(
        current_state_ ? current_state_->getType() : 0,
        target,
        emergency ? "emergency" : "normal",
        true
    );
    
    // Ejecutar onExit del estado actual
    if (current_state_) {
        try {
            logging::log_debug("HFSM", 
                std::string("Ejecutando onExit para: ") + current_state_->getName());
            
            bool exit_success = current_state_->onExit();
            if (!exit_success) {
                logging::log_warn("HFSM", 
                    std::string("onExit devolvió falso para: ") + 
                    current_state_->getName());
            }
        } catch (const std::exception& e) {
            logging::log_error("HFSM", 
                std::string("Error en onExit: ") + e.what(),
                {
                    {"error_code", static_cast<int>(ErrorCode::STATE_TRANSITION_ERROR)},
                    {"state", current_state_->getName()}
                });
            
            if (!emergency) {
                return ErrorCode::STATE_TRANSITION_ERROR;
            }
        }
    }
    
    // Cambiar estado actual
    IState* previous_state = current_state_;
    current_state_ = it->second.get();
    
    // Ejecutar onEnter del nuevo estado
    try {
        logging::log_debug("HFSM", 
            std::string("Ejecutando onEnter para: ") + current_state_->getName());
        
        bool enter_success = current_state_->onEnter();
        if (!enter_success) {
            logging::log_error("HFSM", 
                std::string("onEnter devolvió falso para: ") + 
                current_state_->getName(),
                {{"error_code", static_cast<int>(ErrorCode::STATE_TRANSITION_ERROR)}});
            
            // Revertir si no es emergencia
            if (!emergency && previous_state) {
                current_state_ = previous_state;
                try {
                    logging::log_warn("HFSM", 
                        std::string("Revertiendo a estado: ") + current_state_->getName());
                    current_state_->onEnter();
                } catch (...) {
                    // Fallback a ERROR state
                    auto error_it = states_.find(static_cast<uint32_t>(StateType::ERROR));
                    if (error_it != states_.end()) {
                        current_state_ = error_it->second.get();
                        logging::log_error("HFSM", 
                            "Fallback a estado ERROR");
                        current_state_->onEnter();
                    }
                }
                return ErrorCode::STATE_TRANSITION_ERROR;
            }
        }
        
    } catch (const std::exception& e) {
        logging::log_error("HFSM", 
            std::string("Error en onEnter: ") + e.what(),
            {
                {"error_code", static_cast<int>(ErrorCode::STATE_TRANSITION_ERROR)},
                {"state", current_state_->getName()}
            });
        
        if (!emergency) {
            // Revertir
            if (previous_state) {
                current_state_ = previous_state;
                try {
                    logging::log_warn("HFSM", 
                        std::string("Revertiendo a estado: ") + current_state_->getName());
                    current_state_->onEnter();
                } catch (...) {
                    // Fallback a ERROR state
                    auto error_it = states_.find(static_cast<uint32_t>(StateType::ERROR));
                    if (error_it != states_.end()) {
                        current_state_ = error_it->second.get();
                        logging::log_error("HFSM", "Fallback a estado ERROR");
                        current_state_->onEnter();
                    }
                }
            }
            return ErrorCode::STATE_TRANSITION_ERROR;
        }
    }
    
    // Actualizar temporizador
    last_state_change_ = std::chrono::steady_clock::now();
    stats_.state_transitions++;
    
    // Log de transición exitosa
    if (config_.getLogTransitions()) {
        std::string from_name = current_state_ ? current_state_->getName() : "None";
        std::string to_name = it->second->getName();
        
        logging::log_info("HFSM", 
            std::string("TRANSICIÓN: ") + from_name + " -> " + to_name,
            {
                {"from", from_name},
                {"to", to_name},
                {"emergency", emergency},
                {"elapsed_ms", std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - last_state_change_).count()}
            });
    }
    
    // Publicar evento de cambio de estado
    if (!emergency) {
        try {
            Event event(EventType::STATE_CHANGED, 
                       static_cast<int32_t>(target),
                       "HFSM");
            event_bus_.publish(event);
            logging::log_debug("HFSM", "Evento STATE_CHANGED publicado");
        } catch (...) {
            logging::log_error("HFSM", "Error publicando evento STATE_CHANGED");
        }
    }
    
    // Si es estado final, detener la máquina
    if (current_state_->isFinalState()) {
        logging::log_info("HFSM", 
            std::string("Alcanzó el estado final: ") + current_state_->getName());
        
        if (!shutdown_requested_.load()) {
            gracefulShutdown();
        }
    }
    
    return ErrorCode::SUCCESS;
}

void HFSMCore::gracefulShutdown() {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    
    // Verificar si ya se solicitó shutdown
    if (shutdown_requested_.exchange(true)) {
        logging::log_warn("HFSM", "Shutdown ya solicitado, ignorando solicitud duplicada");
        return;
    }
    
    logging::log_info("HFSM", "Iniciando shutdown ordenado...",
        {{"timeout_ms", config_.getShutdownTimeoutMs()}});
    
    try {
        // 1. Transición a estado CLEANING para limpieza de recursos
        auto cleaning_state = states_.find(static_cast<uint32_t>(StateType::CLEANING));
        if (cleaning_state != states_.end()) {
            // Intentar transición a CLEANING
            ErrorCode transition_result = transitionToInternal(static_cast<uint32_t>(StateType::CLEANING), false);
            
            if (transition_result == ErrorCode::SUCCESS) {
                logging::log_info("HFSM", "Estado CLEANING activado, realizando limpieza...");
                
                // Esperar con timeout configurable para completar limpieza
                auto cleanup_start = std::chrono::steady_clock::now();
                const std::chrono::milliseconds cleanup_timeout(config_.getShutdownTimeoutMs());
                
                while (current_state_ && current_state_->getType() == static_cast<uint32_t>(StateType::CLEANING)) {
                    auto elapsed = std::chrono::steady_clock::now() - cleanup_start;
                    if (elapsed > cleanup_timeout) {
                        logging::log_warn("HFSM", 
                            "Timeout en limpieza de recursos",
                            {
                                {"elapsed_ms", std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count()},
                                {"timeout_ms", config_.getShutdownTimeoutMs()}
                            });
                        
                        // Publicar evento de timeout
                        ErrorInfo timeout_info;
                        timeout_info.error_code = static_cast<uint32_t>(ErrorCode::GRACEFUL_SHUTDOWN_TIMEOUT);
                        timeout_info.description = "Cleanup timeout during graceful shutdown";
                        timeout_info.severity = 2;
                        timeout_info.module = "HFSM";
                        
                        Event timeout_event(EventType::WATCHDOG_TIMEOUT, timeout_info, "HFSM");
                        event_bus_.publish(timeout_event);
                        break;
                    }
                    
                    // Pequeña pausa para evitar consumo excesivo de CPU
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                }
            } else {
                logging::log_warn("HFSM", 
                    "No se pudo transicionar a CLEANING",
                    {{"error_code", static_cast<int>(transition_result)}});
            }
        }
        
        // 2. Notificar a todos los componentes del inicio de shutdown
        logging::log_info("HFSM", "Notificando inicio de shutdown a componentes...");
        Event shutdown_init_event(EventType::SHUTDOWN_INITIATED, 0, "HFSM");
        event_bus_.publish(shutdown_init_event);
        
        // Procesar eventos pendientes de notificación
        for (int i = 0; i < 10; i++) {
            if (!event_bus_.processOne()) break;
        }
        
        // 3. Apagar módulos de manera controlada
        if (module_manager_) {
            logging::log_info("HFSM", "Iniciando shutdown ordenado de módulos...");
            
            try {
                // Apagado no forzoso (elegante)
                module_manager_->shutdownAll(false);
                logging::log_info("HFSM", "Shutdown de módulos completado exitosamente");
                
            } catch (const std::exception& e) {
                logging::log_error("HFSM", 
                    std::string("Error durante shutdown de módulos: ") + e.what(),
                    {{"error_code", static_cast<int>(ErrorCode::MODULE_SHUTDOWN_ERROR)}});
                
                // Intentar apagado forzoso como fallback
                logging::log_warn("HFSM", "Intentando shutdown forzoso de módulos...");
                try {
                    module_manager_->shutdownAll(true);
                    logging::log_info("HFSM", "Shutdown forzoso completado");
                } catch (...) {
                    logging::log_error("HFSM", "Shutdown forzoso de módulos también falló",
                                 {{"error_code", static_cast<int>(ErrorCode::CRITICAL_FAILURE)}});
                }
            }
        }
        
        // 4. Transición a estado SHUTDOWN
        logging::log_info("HFSM", "Transicionando a estado SHUTDOWN...");
        auto shutdown_state = states_.find(static_cast<uint32_t>(StateType::SHUTDOWN));
        if (shutdown_state != states_.end()) {
            ErrorCode shutdown_result = transitionToInternal(static_cast<uint32_t>(StateType::SHUTDOWN), false);
            
            if (shutdown_result != ErrorCode::SUCCESS) {
                logging::log_error("HFSM", 
                    "Fallo transición a SHUTDOWN",
                    {{"error_code", static_cast<int>(ErrorCode::SHUTDOWN_ERROR)}});
                
                // Intentar transición de emergencia
                logging::log_warn("HFSM", "Intentando transición de emergencia a SHUTDOWN");
                transitionToInternal(static_cast<uint32_t>(StateType::SHUTDOWN), true);
            }
        }
        
        // 5. Detener el bucle principal y limpiar recursos
        logging::log_info("HFSM", "Deteniendo bucle principal...");
        running_ = false;
        
        // Limpiar eventos pendientes
        event_bus_.clear();
        
        // 6. Log de estadísticas finales
        logStatistics();
        
        logging::log_info("HFSM", "SHUTDOWN ORDENADO COMPLETADO EXITOSAMENTE");
        
    } catch (const std::exception& e) {
        logging::log_error("HFSM", 
            std::string("ERROR CRÍTICO durante shutdown ordenado: ") + e.what(),
            {{"error_code", static_cast<int>(ErrorCode::SHUTDOWN_ERROR)}});
        
        // Ejecutar shutdown de emergencia como último recurso
        emergencyShutdown();
    }
}

// Métodos públicos
ErrorCode HFSMCore::initialize() {
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    
    if (initialized_.load()) {
        logging::log_warn("HFSM", "Ya inicializado");
        return ErrorCode::ALREADY_INITIALIZED;
    }
    
    if (running_.load()) {
        logging::log_warn("HFSM", "Ya corriendo");
        return ErrorCode::ALREADY_INITIALIZED;
    }
    
    // Verificar que tenemos al menos el estado INITIALIZING
    if (states_.find(static_cast<uint32_t>(StateType::INITIALIZING)) == states_.end()) {
        logging::log_error("HFSM", "INICIALIZANDO estado no registrado",
            {{"error_code", static_cast<int>(ErrorCode::STATE_NOT_FOUND)}});
        return ErrorCode::STATE_NOT_FOUND;
    }
    
    logging::log_info("HFSM", "Inicializando HFSM...",
        {
            {"registered_states", states_.size()},
            {"module_manager", module_manager_ != nullptr}
        });
    
    // Configurar estados
    for (auto& pair : states_) {
        pair.second->setHFSMContext(this);
        logging::log_debug("HFSM", 
            std::string("Contexto configurado para estado: ") + pair.second->getName());
    }
    
    // Iniciar hilo principal
    try {
        running_ = true;
        shutdown_requested_ = false;
        main_thread_ = std::thread(&HFSMCore::runLoop, this);
        
        logging::log_debug("HFSM", "Hilo principal creado");
        
    } catch (const std::exception& e) {
        logging::log_fatal("HFSM", 
            std::string("No se pudo iniciar el hilo principal: ") + e.what(),
            {{"error_code", static_cast<int>(ErrorCode::THREAD_START_FAILED)}});
        
        running_ = false;
        return ErrorCode::THREAD_START_FAILED;
    }
    
    // Transicionar a INITIALIZING
    ErrorCode result = transitionToInternal(static_cast<uint32_t>(StateType::INITIALIZING));
    if (result != ErrorCode::SUCCESS) {
        logging::log_error("HFSM", "Falló la transición inicial a INICIALIZANDO",
            {{"error_code", static_cast<int>(result)}});
        
        running_ = false;
        if (main_thread_.joinable()) {
            main_thread_.join();
        }
        return result;
    }
    
    initialized_ = true;
    logging::log_info("HFSM", "Inicialización completada");
    return ErrorCode::SUCCESS;
}

ErrorCode HFSMCore::shutdown(bool emergency) {
    if (!initialized_.load() && !running_.load()) {
        logging::log_debug("HFSM", "Ya está apagado");
        return ErrorCode::SUCCESS; // Ya está apagado
    }
    
    logging::log_info("HFSM", 
        std::string("Iniciando shutdown: ") + (emergency ? "EMERGENCIA" : "ORDENADO"));
    
    if (emergency) {
        emergencyShutdown();
    } else {
        gracefulShutdown();
    }
    
    // Esperar a que el hilo principal termine
    if (main_thread_.joinable()) {
        auto timeout = emergency ? 
            std::chrono::milliseconds(100) : 
            std::chrono::milliseconds(config_.getShutdownTimeoutMs());
        
        auto start = std::chrono::steady_clock::now();
        
        while (std::chrono::steady_clock::now() - start < timeout) {
            if (!main_thread_.joinable()) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        
        if (main_thread_.joinable()) {
            logging::log_warn("HFSM", "El hilo principal no se detuvo con gracia y se separó.");
            main_thread_.detach();
        } else {
            logging::log_debug("HFSM", "Hilo principal terminado correctamente");
        }
    }
    
    // Limpiar
    event_bus_.clear();
    initialized_ = false;
    
    logging::log_info("HFSM", "Apagado completo");
    return ErrorCode::SUCCESS;
}

ErrorCode HFSMCore::reset() {
    logging::log_info("HFSM", "Reiniciando sistema...");
    
    shutdown(true);
    
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    
    // Resetear todo
    current_state_ = nullptr;
    running_ = false;
    shutdown_requested_ = false;
    initialized_ = false;
    last_state_change_ = std::chrono::steady_clock::now();
    last_update_ = std::chrono::steady_clock::now();
    
    // Resetear estadísticas
    stats_ = {};
    
    logging::log_info("HFSM", "Reinicio completo");
    return ErrorCode::SUCCESS;
}

ErrorCode HFSMCore::registerState(std::unique_ptr<IState> state) {
    if (!state) {
        logging::log_error("HFSM", "No se puede registrar el estado nulo",
            {{"error_code", static_cast<int>(ErrorCode::INVALID_PARAMETER)}});
        return ErrorCode::INVALID_PARAMETER;
    }
    
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    
    uint32_t type = state->getType();
    std::string name = state->getName();
    
    if (states_.find(type) != states_.end()) {
        logging::log_error("HFSM", 
            std::string("Estado ya registrado: ") + name,
            {
                {"error_code", static_cast<int>(ErrorCode::INVALID_PARAMETER)},
                {"state_type", type},
                {"state_name", name}
            });
        return ErrorCode::INVALID_PARAMETER;
    }
    
    // Asignar contexto si ya estamos inicializados
    if (initialized_.load()) {
        state->setHFSMContext(this);
    }
    
    states_[type] = std::move(state);
    
    logging::log_info("HFSM", 
        std::string("Estado registrado: ") + name,
        {
            {"state_type", type},
            {"state_name", name},
            {"total_states", states_.size()}
        });
    
    return ErrorCode::SUCCESS;
}

ErrorCode HFSMCore::transitionTo(uint32_t target) {
    if (shutdown_requested_.load()) {
        logging::log_warn("HFSM", "Transición rechazada: cierre en curso");
        return ErrorCode::SHUTDOWN_IN_PROGRESS;
    }
    
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return transitionToInternal(target, false);
}

void HFSMCore::publishEvent(const Event& event) {
    if (config_.getLogEvents()) {
        std::string type_str = "UNKNOWN";
        try {
            type_str = eventTypeToString(event.type);
        } catch (...) {
            type_str = std::to_string(static_cast<uint32_t>(event.type));
        }
        
        logging::log_debug("HFSM", 
            std::string("Evento publicado: ") + type_str,
            {
                {"event_type", static_cast<uint32_t>(event.type)},
                {"source", event.source_module},
                {"has_data", event.hasData()}
            });
    }
    
    event_bus_.publish(event);
}

// Métodos de acceso
uint32_t HFSMCore::getCurrentStateType() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return current_state_ ? current_state_->getType() : static_cast<uint32_t>(StateType::UNINITIALIZED);
}

std::string HFSMCore::getCurrentStateName() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return current_state_ ? current_state_->getName() : "";
}

bool HFSMCore::isRunning() const { 
    return running_.load(); 
}

bool HFSMCore::isInitialized() const { 
    return initialized_.load(); 
}

EventBus& HFSMCore::getEventBus() { 
    return event_bus_; 
}

ModuleManager* HFSMCore::getModuleManager() const { 
    return module_manager_; 
}

void HFSMCore::setModuleManager(ModuleManager* manager) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    module_manager_ = manager;
    
    logging::log_info("HFSM", 
        std::string("ModuleManager ") + (manager ? "asignado" : "eliminado"));
}

HFSMConfig HFSMCore::getConfig() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return config_;
}

ErrorCode HFSMCore::updateConfig(const HFSMConfig& new_config) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    
    if (running_.load()) {
        logging::log_warn("HFSM", "No se puede actualizar la configuración mientras se está ejecutando");
        return ErrorCode::INVALID_OPERATION;
    }
    
    config_ = new_config;
    config_.syncAll();
    
    logging::log_info("HFSM", "Configuración actualizada",
        {
            {"state_timeout_ms", config_.getStateTimeoutMs()},
            {"watchdog_enabled", config_.getEnableWatchdog()},
            {"loop_sleep_ms", config_.getLoopSleepMs()}
        });
    
    return ErrorCode::SUCCESS;
}

nlohmann::json HFSMCore::getStatistics() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    auto bus_stats = event_bus_.getStats();
    
    return {
        {"loops", stats_.total_loops},
        {"events_processed", stats_.events_processed},
        {"state_transitions", stats_.state_transitions},
        {"errors", stats_.errors},
        {"pending_events", bus_stats.pending_events},
        {"current_state", getCurrentStateName()},
        {"running", isRunning()},
        {"initialized", isInitialized()},
        {"registered_states", states_.size()}
    };
}

bool HFSMCore::isStateRegistered(uint32_t state_type) const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return states_.find(state_type) != states_.end();
}

std::vector<uint32_t> HFSMCore::getRegisteredStates() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    std::vector<uint32_t> registered_states;
    for (const auto& pair : states_) {
        registered_states.push_back(pair.first);
    }
    return registered_states;
}

bool HFSMCore::isStateTransitionPossible(uint32_t from_state, uint32_t to_state) const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    
    // Verificar que ambos estados existen
    auto from_it = states_.find(from_state);
    auto to_it = states_.find(to_state);
    
    if (from_it == states_.end() || to_it == states_.end()) {
        return false;
    }
    
    // Consultar si la transición es válida según el estado origen
    return from_it->second->canTransitionTo(to_state);
}

bool HFSMCore::isStateReachableFromCurrent(uint32_t target_state) const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    
    if (!current_state_) {
        return false;
    }
    
    // Verificar que el estado objetivo existe
    auto target_it = states_.find(target_state);
    if (target_it == states_.end()) {
        return false;
    }
    
    // Consultar si podemos transicionar desde el estado actual
    return current_state_->canTransitionTo(target_state);
}

} // namespace ns_fsm
