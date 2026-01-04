// hfsm_states.hpp - ESTADOS DE HFSM SIN DEPENDENCIAS CIRCULARES
#ifndef HFSM_STATES_FIXED_HPP
#define HFSM_STATES_FIXED_HPP

#include "hfsm_core.hpp"
#include "hfsm_event_types.hpp"
#include "hardware/utils/Diagnostics.h"

#include <chrono>
#include <functional>
#include <nlohmann/json.hpp>
#include <random>

namespace ns_fsm {

// ============================================================================
// ESTADO BASE MEJORADO
// ============================================================================

class BaseState : public IState {
protected:
    uint32_t type_;
    std::string name_;
    void* context_{nullptr};
    std::chrono::milliseconds timeout_;
    
    std::function<void()> on_enter_callback_;
    std::function<void()> on_exit_callback_;
    std::function<void()> update_callback_;
    
public:
    BaseState(uint32_t type, const std::string& name, 
              std::chrono::milliseconds timeout = std::chrono::milliseconds(5000))
        : type_(type), name_(name), timeout_(timeout) {}
    
    virtual ~BaseState() = default;
    
    // Implementación de IState
    uint32_t getType() const override { return type_; }
    std::string getName() const override { return name_; }
    
    void setHFSMContext(void* context) override {
        context_ = context;
    }
    
    void setOnEnterCallback(std::function<void()> callback) {
        on_enter_callback_ = callback;
    }
    
    void setOnExitCallback(std::function<void()> callback) {
        on_exit_callback_ = callback;
    }
    
    void setUpdateCallback(std::function<void()> callback) {
        update_callback_ = callback;
    }
    
    bool onEnter() override {
        if (context_) {
            HAL_INFO("State", std::string("Entrando: ") + getName());
        }
        if (on_enter_callback_) {
            on_enter_callback_();
        }
        return true;
    }
    
    bool onExit() override {
        if (context_) {
            HAL_INFO("State", std::string("Saliendo: ") + getName());
        }
        if (on_exit_callback_) {
            on_exit_callback_();
        }
        return true;
    }
    
    void update() override {
        if (update_callback_) {
            update_callback_();
        }
    }
    
    virtual bool handleEvent(const Event& event) override {
        // Por defecto, no maneja eventos
        (void)event;
        return false;
    }
    
    std::chrono::milliseconds getTimeout() const override {
        return timeout_;
    }
    
    virtual bool isFinalState() const override {
        return false;
    }
    
    virtual bool canTransitionTo(uint32_t target) const override {
        StateType current = static_cast<StateType>(type_);
        StateType next = static_cast<StateType>(target);
        
        // Permitir transiciones basadas en el tipo de estado actual
        switch (current) {
            case StateType::UNINITIALIZED:
                return next == StateType::INITIALIZING;
            
            case StateType::INITIALIZING:
                return next == StateType::READY || 
                       next == StateType::ERROR || 
                       next == StateType::SHUTDOWN;
            
            case StateType::READY:
                return next == StateType::OPERATIONAL || 
                       next == StateType::IDLE || 
                       next == StateType::ERROR || 
                       next == StateType::SHUTDOWN;
            
            case StateType::OPERATIONAL:
                return next == StateType::READY || 
                       next == StateType::PAUSED || 
                       next == StateType::ERROR || 
                       next == StateType::SHUTDOWN;
            
            case StateType::ERROR:
                return next == StateType::RECOVERING || 
                       next == StateType::SHUTDOWN;
            
            case StateType::RECOVERING:
                return next == StateType::READY || 
                       next == StateType::ERROR || 
                       next == StateType::SHUTDOWN;
            
            case StateType::SHUTDOWN:
                return false; // Estado final
            
            case StateType::IDLE:
                return next == StateType::READY || 
                       next == StateType::ERROR || 
                       next == StateType::SHUTDOWN;
            
            case StateType::PAUSED:
                return next == StateType::OPERATIONAL || 
                       next == StateType::ERROR || 
                       next == StateType::SHUTDOWN;
            
            case StateType::CLEANING:
                return next == StateType::SHUTDOWN || 
                       next == StateType::ERROR;
            
            default:
                // Para estados personalizados, permitir todas las transiciones por defecto
                return true;
        }
    }
};

// ============================================================================
// ESTADOS ESPECÍFICOS IMPLEMENTADOS
// ============================================================================

class UninitializedState : public BaseState {
public:
    UninitializedState() 
        : BaseState(static_cast<uint32_t>(StateType::UNINITIALIZED), 
                   "UNINITIALIZED", 
                   std::chrono::milliseconds(1000)) {}
    
    bool canTransitionTo(uint32_t target) const override {
        // Sólo puede transicionar a INITIALIZING
        return static_cast<StateType>(target) == StateType::INITIALIZING;
    }
};

class InitializingState : public BaseState {
public:
    InitializingState() 
        : BaseState(static_cast<uint32_t>(StateType::INITIALIZING), 
                   "INITIALIZING", 
                   std::chrono::milliseconds(10000)) {}
    
    bool onEnter() override {
        BaseState::onEnter();
        HAL_INFO("Initializing", "Comenzando la inicialización del sistema...");
        return true;
    }
    
    void update() override {
        // Simular inicialización que toma un poco de tiempo
        static int count = 0;
        if (count++ >= 5) { // Después de 5 updates, considerar inicialización completa
            HFSMCore* hfsm = static_cast<HFSMCore*>(context_);
            if (hfsm) {
                HAL_INFO("Initializing", "Inicialización completada");
                // Transicionar automáticamente a READY
                hfsm->transitionTo(static_cast<uint32_t>(StateType::READY));
            }
        }
    }
    
    bool canTransitionTo(uint32_t target) const override {
        StateType next = static_cast<StateType>(target);
        return next == StateType::READY || 
               next == StateType::ERROR || 
               next == StateType::SHUTDOWN;
    }
};

class ReadyState : public BaseState {
public:
    ReadyState() 
        : BaseState(static_cast<uint32_t>(StateType::READY), 
                   "READY", 
                   std::chrono::milliseconds(30000)) {}
    
    bool onEnter() override {
        BaseState::onEnter();
        HAL_INFO("Ready", "Sistema listo para operar");
        return true;
    }
    
    bool canTransitionTo(uint32_t target) const override {
        StateType next = static_cast<StateType>(target);
        return next == StateType::OPERATIONAL || 
               next == StateType::IDLE || 
               next == StateType::ERROR || 
               next == StateType::SHUTDOWN;
    }
};

class OperationalState : public BaseState {
public:
    OperationalState() 
        : BaseState(static_cast<uint32_t>(StateType::OPERATIONAL), 
                   "OPERATIONAL", 
                   std::chrono::milliseconds(60000)) {}
    
    bool onEnter() override {
        BaseState::onEnter();
        HAL_INFO("Operational", "Sistema en modo operacional");
        return true;
    }
    
    bool canTransitionTo(uint32_t target) const override {
        StateType next = static_cast<StateType>(target);
        return next == StateType::READY || 
               next == StateType::PAUSED || 
               next == StateType::ERROR || 
               next == StateType::SHUTDOWN;
    }
};

class ErrorState : public BaseState {
public:
    ErrorState() 
        : BaseState(static_cast<uint32_t>(StateType::ERROR), 
                   "ERROR", 
                   std::chrono::milliseconds(30000)) {}
    
    bool onEnter() override {
        BaseState::onEnter();
        HAL_ERROR("Error", "Estado de error detectado", 1000);
        return true;
    }
    
    bool canTransitionTo(uint32_t target) const override {
        StateType next = static_cast<StateType>(target);
        return next == StateType::RECOVERING || 
               next == StateType::SHUTDOWN;
    }
};

class RecoveringState : public BaseState {
public:
    RecoveringState() 
        : BaseState(static_cast<uint32_t>(StateType::RECOVERING), 
                   "RECOVERING", 
                   std::chrono::milliseconds(15000)) {}
    
    bool onEnter() override {
        BaseState::onEnter();
        HAL_INFO("Recovering", "Iniciando proceso de recuperación...");
        return true;
    }
    
    void update() override {
        // Simular recuperación que toma tiempo
        static int count = 0;
        if (count++ >= 10) { // Después de 10 updates, recuperación completa
            HFSMCore* hfsm = static_cast<HFSMCore*>(context_);
            if (hfsm) {
                HAL_INFO("Recovering", "Recuperación completada");
                hfsm->transitionTo(static_cast<uint32_t>(StateType::READY));
            }
        }
    }
    
    bool canTransitionTo(uint32_t target) const override {
        StateType next = static_cast<StateType>(target);
        return next == StateType::READY || 
               next == StateType::ERROR || 
               next == StateType::SHUTDOWN;
    }
};

class ShutdownState : public BaseState {
public:
    ShutdownState() 
        : BaseState(static_cast<uint32_t>(StateType::SHUTDOWN), 
                   "SHUTDOWN", 
                   std::chrono::milliseconds(5000)) {}
    
    bool onEnter() override {
        BaseState::onEnter();
        HAL_INFO("Shutdown", "Apagado del sistema iniciado");
        return true;
    }
    
    bool isFinalState() const override {
        return true;
    }
    
    bool canTransitionTo(uint32_t target) const override {
        // Estado final - no puede transicionar a ningún otro estado
        (void)target;
        return false;
    }
};

class IdleState : public BaseState {
public:
    IdleState() 
        : BaseState(static_cast<uint32_t>(StateType::IDLE), 
                   "IDLE", 
                   std::chrono::milliseconds(1800000)) {} // 30 minutos
    
    bool onEnter() override {
        BaseState::onEnter();
        HAL_INFO("Idle", "Sistema en modo inactivo");
        return true;
    }
    
    bool canTransitionTo(uint32_t target) const override {
        StateType next = static_cast<StateType>(target);
        return next == StateType::READY || 
               next == StateType::ERROR || 
               next == StateType::SHUTDOWN;
    }
};

class PausedState : public BaseState {
public:
    PausedState() 
        : BaseState(static_cast<uint32_t>(StateType::PAUSED), 
                   "PAUSED", 
                   std::chrono::milliseconds(300000)) {} // 5 minutos
    
    bool onEnter() override {
        BaseState::onEnter();
        HAL_INFO("Paused", "Sistema en pausa");
        return true;
    }
    
    bool canTransitionTo(uint32_t target) const override {
        StateType next = static_cast<StateType>(target);
        return next == StateType::OPERATIONAL || 
               next == StateType::ERROR || 
               next == StateType::SHUTDOWN;
    }
};

class CleaningState : public BaseState {
public:
    CleaningState() 
        : BaseState(static_cast<uint32_t>(StateType::CLEANING), 
                   "CLEANING", 
                   std::chrono::milliseconds(5000)) {}
    
    bool onEnter() override {
        BaseState::onEnter();
        HAL_INFO("Cleaning", "Iniciando limpieza del sistema para shutdown elegante...");
        return true;
    }
    
    void update() override {
        // Simular limpieza paso a paso
        static int step = 0;
        
        switch (++step) {
            case 1: HAL_INFO("Cleaning", "Paso 1/3: Limpiando buffers"); break;
            case 3: HAL_INFO("Cleaning", "Paso 2/3: Liberando recursos"); break;
            case 5: 
                HAL_INFO("Cleaning", "Paso 3/3: Limpieza completada");
                // Enviar evento usando el método disponible
                HFSMCore* hfsm = static_cast<HFSMCore*>(context_);
                if (hfsm) {
                    Event event(EventType::TASK_COMPLETE, 0, "CleaningState");
                    hfsm->publishEvent(event);
                }
                break;
        }
    }

    bool handleEvent(const Event& event) override {
        switch (event.type) {
            case EventType::TASK_COMPLETE: {
                HFSMCore* hfsm = static_cast<HFSMCore*>(context_);
                if (hfsm) {
                    hfsm->transitionTo(static_cast<uint32_t>(StateType::SHUTDOWN));
                    return true;
                }
                return false;
            }
                
            case EventType::SHUTDOWN_REQUEST: {
                HFSMCore* hfsm = static_cast<HFSMCore*>(context_);
                if (hfsm) {
                    hfsm->transitionTo(static_cast<uint32_t>(StateType::SHUTDOWN));
                    return true;
                }
                return false;
            }
                
            default:
                return BaseState::handleEvent(event);
        }
    }
    
    bool canTransitionTo(uint32_t target) const override {
        StateType next = static_cast<StateType>(target);
        return next == StateType::SHUTDOWN || 
               next == StateType::ERROR;
    }
};

class DegradedState : public BaseState {
private:
    uint32_t recovery_attempts_{0};
    
public:
    DegradedState() 
        : BaseState(static_cast<uint32_t>(StateType::DEGRADED), 
                   "DEGRADED", 
                   std::chrono::milliseconds(60000)) {}
    
    bool onEnter() override {
        BaseState::onEnter();
        HAL_WARN("Degraded", "Sistema en modo degradado - funcionalidad limitada");
        return true;
    }
    
    void update() override {
        BaseState::update();
        
        // Intentar recuperación periódica
        static uint32_t update_counter = 0;
        if (++update_counter % 10 == 0) { // Cada 10 updates
            HFSMCore* hfsm = static_cast<HFSMCore*>(context_);
            if (hfsm) {
                recovery_attempts_++;
                
                if (recovery_attempts_ >= 5) {
                    Event recovery_event(EventType::RECOVERY_REQUEST, 0, "DegradedState");
                    hfsm->publishEvent(recovery_event);
                }
            }
        }
    }
    
    bool handleEvent(const Event& event) override {
        HFSMCore* hfsm = static_cast<HFSMCore*>(context_);
        if (!hfsm) return false;
        
        switch (event.type) {
            case EventType::COMPONENT_RECOVERED:  // ✅ EVENTO GENÉRICO
                hfsm->transitionTo(static_cast<uint32_t>(StateType::OPERATIONAL));
                return true;
                
            case EventType::EMERGENCY_STOP:  // ✅ EVENTO GENÉRICO
                hfsm->transitionTo(static_cast<uint32_t>(StateType::ERROR));
                return true;
                
            default:
                return BaseState::handleEvent(event);
        }
    }
    
    bool canTransitionTo(uint32_t target) const override {
        StateType next = static_cast<StateType>(target);
        return next == StateType::OPERATIONAL || 
               next == StateType::ERROR || 
               next == StateType::SHUTDOWN;
    }
};

// ============================================================================
// ESTADOS AVANZADOS DE ROBÓTICA
// ============================================================================

class NavigatingState : public BaseState {
private:
    bool waiting_for_navigation_complete_{false};
    bool obstacle_detected_{false};
    
public:
    NavigatingState() 
        : BaseState(static_cast<uint32_t>(StateType::NAVIGATING_TO_GOAL), 
                   "NAVIGATING_TO_GOAL", 
                   std::chrono::milliseconds(30000)) {}
    
    bool onEnter() override {
        BaseState::onEnter();
        HAL_INFO("Navigating", "Iniciando navegación hacia objetivo");
        
        // La HFSM no accede a hardware directamente
        // Solo publica eventos para que los módulos actúen
        HFSMCore* hfsm = static_cast<HFSMCore*>(context_);
        if (hfsm) {
            Event start_event(EventType::ROBOT_NAVIGATION_START,
                             "Solicitud de inicio de navegación",
                             "HFSM");
            hfsm->publishEvent(start_event);
            waiting_for_navigation_complete_ = true;
        }
        
        return true;
    }
    
    void update() override {
        BaseState::update();
        
        // La HFSM no monitorea sensores directamente
        // Solo maneja eventos de los módulos
        // NO HAY ACCESO A HARDWARE AQUÍ
    }
    
    bool handleEvent(const Event& event) override {
        HFSMCore* hfsm = static_cast<HFSMCore*>(context_);
        if (!hfsm) return false;
        
        switch (event.type) {
            case EventType::ROBOT_NAVIGATION_COMPLETE: {
                HAL_INFO("Navigating", "Navegación completada por módulo");
                waiting_for_navigation_complete_ = false;
                hfsm->transitionTo(static_cast<uint32_t>(StateType::READY));
                return true;
            }
                
            case EventType::ROBOT_COLLISION_DETECTED: {
                HAL_INFO("Navigating", "Obstáculo detectado por módulo de percepción");
                obstacle_detected_ = true;
                // La HFSM no decide cómo evitar obstáculos
                // Solo transiciona al estado apropiado
                hfsm->transitionTo(static_cast<uint32_t>(StateType::AVOID_OBSTACLE));
                return true;
            }
                
            case EventType::EMERGENCY_STOP: {
                HAL_WARN("Navigating", "Parada de emergencia durante navegación");
                waiting_for_navigation_complete_ = false;
                hfsm->transitionTo(static_cast<uint32_t>(StateType::ERROR));
                return true;
            }
                
            default:
                return BaseState::handleEvent(event);
        }
    }
    
    bool canTransitionTo(uint32_t target) const override {
        StateType next = static_cast<StateType>(target);
        return next == StateType::READY ||
               next == StateType::AVOID_OBSTACLE ||
               next == StateType::ERROR ||
               next == StateType::SHUTDOWN;
    }
};

class ManipulatingState : public BaseState {
private:
    bool waiting_for_completion_{false};
    
public:
    ManipulatingState() 
        : BaseState(static_cast<uint32_t>(StateType::MANIPULATING), 
                   "MANIPULATING", 
                   std::chrono::milliseconds(15000)) {}
    
    bool onEnter() override {
        BaseState::onEnter();
        HAL_INFO("Manipulating", "Iniciando manipulación de objeto");
        
        // La HFSM no controla directamente el brazo robótico
        // Solo notifica a los módulos
        HFSMCore* hfsm = static_cast<HFSMCore*>(context_);
        if (hfsm) {
            Event start_event(EventType::ROBOT_MANIPULATION_START,
                             "Solicitud de inicio de manipulación",
                             "HFSM");
            hfsm->publishEvent(start_event);
            waiting_for_completion_ = true;
        }
        
        return true;
    }
    
    void update() override {
        BaseState::update();
        
        // La HFSM no monitorea sensores de fuerza ni posición
        // Solo maneja eventos de los módulos de control
    }
    
    bool handleEvent(const Event& event) override {
        HFSMCore* hfsm = static_cast<HFSMCore*>(context_);
        if (!hfsm) return false;
        
        switch (event.type) {
            case EventType::ROBOT_MANIPULATION_COMPLETE: {
                HAL_INFO("Manipulating", "Manipulación completada por módulo");
                waiting_for_completion_ = false;
                hfsm->transitionTo(static_cast<uint32_t>(StateType::READY));
                return true;
            }
                
            case EventType::ROBOT_VISION_OBJECT_DETECTED: {
                // Solo registrar, no actuar directamente
                HAL_DEBUG("Manipulating", "Objeto detectado por módulo de visión");
                return true;
            }
                
            case EventType::EMERGENCY_STOP: {
                HAL_WARN("Manipulating", "Parada de emergencia durante manipulación");
                waiting_for_completion_ = false;
                hfsm->transitionTo(static_cast<uint32_t>(StateType::ERROR));
                return true;
            }
                
            default:
                return BaseState::handleEvent(event);
        }
    }
    
    bool canTransitionTo(uint32_t target) const override {
        StateType next = static_cast<StateType>(target);
        return next == StateType::READY ||
               next == StateType::ERROR ||
               next == StateType::SHUTDOWN;
    }
};

class ChargingState : public BaseState {
private:
    bool waiting_for_charge_complete_{false};
    
public:
    ChargingState() 
        : BaseState(static_cast<uint32_t>(StateType::CHARGING), 
                   "CHARGING", 
                   std::chrono::milliseconds(1800000)) {}
    
    bool onEnter() override {
        BaseState::onEnter();
        HAL_INFO("Charging", "Iniciando proceso de carga");
        
        // La HFSM no monitorea el nivel de batería directamente
        // Solo notifica al módulo de gestión de energía
        HFSMCore* hfsm = static_cast<HFSMCore*>(context_);
        if (hfsm) {
            Event start_event(EventType::ROBOT_CHARGING_START,
                             "Solicitud de inicio de carga",
                             "HFSM");
            hfsm->publishEvent(start_event);
            waiting_for_charge_complete_ = true;
        }
        
        return true;
    }
    
    void update() override {
        BaseState::update();
        
        // La HFSM NO monitorea sensores de batería
        // Esto lo hace el módulo de gestión de energía
    }
    
    bool handleEvent(const Event& event) override {
        HFSMCore* hfsm = static_cast<HFSMCore*>(context_);
        if (!hfsm) return false;
        
        switch (event.type) {
            case EventType::ROBOT_CHARGING_COMPLETE: {
                HAL_INFO("Charging", "Carga completada por módulo de energía");
                waiting_for_charge_complete_ = false;
                hfsm->transitionTo(static_cast<uint32_t>(StateType::READY));
                return true;
            }
                
            case EventType::EMERGENCY_STOP: {
                HAL_WARN("Charging", "Parada de emergencia durante carga");
                waiting_for_charge_complete_ = false;
                hfsm->transitionTo(static_cast<uint32_t>(StateType::ERROR));
                return true;
            }
                
            case EventType::LOW_BATTERY: {
                // Solo registrar evento, el módulo de energía maneja esto
                HAL_DEBUG("Charging", "Evento de batería baja recibido");
                return true;
            }
                
            default:
                return BaseState::handleEvent(event);
        }
    }
    
    bool canTransitionTo(uint32_t target) const override {
        StateType next = static_cast<StateType>(target);
        return next == StateType::READY ||
               next == StateType::ERROR ||
               next == StateType::SHUTDOWN;
    }
};

class FleetCoordinationState : public BaseState {
private:
    bool coordination_active_{false};
    
public:
    FleetCoordinationState() 
        : BaseState(static_cast<uint32_t>(StateType::FLEET_COORDINATION), 
                   "FLEET_COORDINATION", 
                   std::chrono::milliseconds(30000)) {}
    
    bool onEnter() override {
        BaseState::onEnter();
        HAL_INFO("FleetCoordination", "Iniciando coordinación de flota");
        
        // La HFSM no se comunica directamente con otros robots
        // Esto lo hace el módulo de coordinación
        HFSMCore* hfsm = static_cast<HFSMCore*>(context_);
        if (hfsm) {
            Event start_event(EventType::ROBOT_FLEET_COORDINATION,
                             "Solicitud de inicio de coordinación de flota",
                             "HFSM");
            hfsm->publishEvent(start_event);
            coordination_active_ = true;
        }
        
        return true;
    }
    
    void update() override {
        BaseState::update();
        
        // La HFSM no maneja comunicación inter-robot
        // Esto lo hace el módulo de coordinación
    }
    
    bool handleEvent(const Event& event) override {
        HFSMCore* hfsm = static_cast<HFSMCore*>(context_);
        if (!hfsm) return false;
        
        switch (event.type) {
            case EventType::TASK_COMPLETE: {
                HAL_INFO("FleetCoordination", "Coordinación completada por módulo");
                coordination_active_ = false;
                hfsm->transitionTo(static_cast<uint32_t>(StateType::READY));
                return true;
            }
                
            case EventType::ROBOT_FORMATION_BREAK: {
                HAL_WARN("FleetCoordination", "Formación rota - notificado por módulo");
                // El módulo de coordinación maneja la recuperación
                return true;
            }
                
            case EventType::EMERGENCY_STOP: {
                HAL_WARN("FleetCoordination", "Parada de emergencia en flota");
                coordination_active_ = false;
                hfsm->transitionTo(static_cast<uint32_t>(StateType::ERROR));
                return true;
            }
                
            default:
                return BaseState::handleEvent(event);
        }
    }
    
    bool canTransitionTo(uint32_t target) const override {
        StateType next = static_cast<StateType>(target);
        return next == StateType::READY ||
               next == StateType::ERROR ||
               next == StateType::SHUTDOWN;
    }
};

class LearningState : public BaseState {
private:
    bool learning_active_{false};
    
public:
    LearningState() 
        : BaseState(static_cast<uint32_t>(StateType::LEARNING), 
                   "LEARNING", 
                   std::chrono::milliseconds(60000)) {}
    
    bool onEnter() override {
        BaseState::onEnter();
        HAL_INFO("Learning", "Iniciando proceso de aprendizaje");
        
        // La HFSM no ejecuta algoritmos de aprendizaje
        // Solo notifica al módulo de aprendizaje
        HFSMCore* hfsm = static_cast<HFSMCore*>(context_);
        if (hfsm) {
            Event start_event(EventType::ROBOT_LEARNING_START,
                             "Solicitud de inicio de aprendizaje",
                             "HFSM");
            hfsm->publishEvent(start_event);
            learning_active_ = true;
        }
        
        return true;
    }
    
    void update() override {
        BaseState::update();
        
        // La HFSM no ejecuta algoritmos ML ni procesa datos
        // Esto lo hace el módulo de aprendizaje
    }
    
    bool handleEvent(const Event& event) override {
        HFSMCore* hfsm = static_cast<HFSMCore*>(context_);
        if (!hfsm) return false;
        
        switch (event.type) {
            case EventType::ROBOT_LEARNING_COMPLETE: {
                HAL_INFO("Learning", "Proceso de aprendizaje completado por módulo");
                learning_active_ = false;
                hfsm->transitionTo(static_cast<uint32_t>(StateType::READY));
                return true;
            }
                
            case EventType::ROBOT_ADAPTIVE_BEHAVIOR: {
                HAL_INFO("Learning", "Comportamiento adaptativo activado por módulo");
                return true;
            }
                
            case EventType::EMERGENCY_STOP: {
                HAL_WARN("Learning", "Parada de emergencia durante aprendizaje");
                learning_active_ = false;
                hfsm->transitionTo(static_cast<uint32_t>(StateType::ERROR));
                return true;
            }
                
            default:
                return BaseState::handleEvent(event);
        }
    }
    
    bool canTransitionTo(uint32_t target) const override {
        StateType next = static_cast<StateType>(target);
        return next == StateType::READY ||
               next == StateType::ERROR ||
               next == StateType::SHUTDOWN;
    }
};

} // namespace ns_fsm

#endif // HFSM_STATES_FIXED_HPP
