// hfsm_builder_fixed.hpp - BUILDER SIN DEPENDENCIAS CIRCULARES
#ifndef HFSM_BUILDER_FIXED_HPP
#define HFSM_BUILDER_FIXED_HPP

#include "hfsm_core.hpp"
#include "hfsm_states.hpp"
#include "hfsm_event_types.hpp"
#include <mutex>
#include <memory>
#include <functional>
#include <iostream>
#include <vector>
#include <string>
#include <unordered_map>
#include <algorithm>

namespace ns_fsm {

// ============================================================================
// FACTORY DE ESTADOS
// ============================================================================

class StateFactory {
public:
    static std::unique_ptr<IState> createState(uint32_t type);
    static std::string stateTypeToString(uint32_t type);
};

// ============================================================================
// BUILDER PRINCIPAL
// ============================================================================

class HFSMBuilder {
private:
    struct TransitionConfig {
        uint32_t from;
        uint32_t to;
        uint32_t trigger;
        std::function<bool()> guard;
        std::string id;
    };
    
    struct StateCallback {
        std::function<void()> on_enter;
        std::function<void()> on_exit;
        std::function<void()> on_update;
    };
    
public:
    // ------------------------------------------------------------------------
    // CONSTRUCTORES Y OPERADORES
    // ------------------------------------------------------------------------
    HFSMBuilder() : config_(HFSMConfig()) {
        config_.syncAll();
    }
    
    // Move constructor
    HFSMBuilder(HFSMBuilder&& other) noexcept
        : hfsm_(std::move(other.hfsm_))
        , config_(std::move(other.config_))
        , state_factory_(std::move(other.state_factory_))
        , states_registered_(std::move(other.states_registered_))
        , transitions_(std::move(other.transitions_))
        , state_callbacks_(std::move(other.state_callbacks_)) {
    }
    
    // Move assignment operator
    HFSMBuilder& operator=(HFSMBuilder&& other) noexcept {
        if (this != &other) {
            hfsm_ = std::move(other.hfsm_);
            config_ = std::move(other.config_);
            state_factory_ = std::move(other.state_factory_);
            states_registered_ = std::move(other.states_registered_);
            transitions_ = std::move(other.transitions_);
            state_callbacks_ = std::move(other.state_callbacks_);
        }
        return *this;
    }
    
    HFSMBuilder(const HFSMBuilder&) = delete;
    HFSMBuilder& operator=(const HFSMBuilder&) = delete;

    // ------------------------------------------------------------------------
    // MÉTODOS DE CONFIGURACIÓN PRINCIPALES
    // ------------------------------------------------------------------------
    HFSMBuilder& forTesting() {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        config_.state_timeout_ms = 500;
        config_.global_timeout_ms = 5000;
        config_.event_processing_timeout_ms = 50;
        config_.enable_watchdog = false;
        config_.loop_sleep_ms = 5;
        config_.max_consecutive_errors = 5;
        config_.shutdown_timeout_ms = 1000;
        config_.log_transitions = true;
        config_.log_events = true;
        config_.syncAll();
        return *this;
    }
    
    HFSMBuilder& forRobotics() {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        config_.state_timeout_ms = 1000;
        config_.global_timeout_ms = 30000;
        config_.event_processing_timeout_ms = 100;
        config_.enable_watchdog = true;
        config_.loop_sleep_ms = 10;
        config_.max_consecutive_errors = 10;
        config_.shutdown_timeout_ms = 3000;
        config_.log_transitions = true;
        config_.log_events = false;
        config_.syncAll();
        return *this;
    }
    
    HFSMBuilder& forProduction() {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        config_.state_timeout_ms = 2000;
        config_.global_timeout_ms = 60000;
        config_.event_processing_timeout_ms = 150;
        config_.enable_watchdog = true;
        config_.loop_sleep_ms = 20;
        config_.max_consecutive_errors = 20;
        config_.shutdown_timeout_ms = 5000;
        config_.log_transitions = true;
        config_.log_events = false;
        config_.syncAll();
        return *this;
    }
    
    HFSMBuilder& withDefaultState(uint32_t type) {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        
        // Verificar si ya está registrado
        if (std::find(states_registered_.begin(), states_registered_.end(), type) 
            != states_registered_.end()) {
            return *this;
        }
        
        auto state = createState(type);
        if (state) {
            ensureHFSMCreated();
            ErrorCode result = hfsm_->registerState(std::move(state));
            if (result == ErrorCode::SUCCESS) {
                states_registered_.push_back(type);
            } else {
                HAL_WARN("HFSM", 
                    std::string("No se pudo registrar estado: ") +
                    StateFactory::stateTypeToString(type) + 
                    " | Error: " + std::to_string(static_cast<int>(result)));
            }
        }
        return *this;
    }
    
    HFSMBuilder& withDefaultState(StateType type) {
        return withDefaultState(static_cast<uint32_t>(type));
    }
    
    HFSMBuilder& withEssentialStates() {
        std::vector<StateType> essential_states = {
            StateType::UNINITIALIZED,
            StateType::INITIALIZING,
            StateType::READY,
            StateType::ERROR,
            StateType::SHUTDOWN
        };
        
        for (auto state_type : essential_states) {
            withDefaultState(state_type);
        }
        return *this;
    }
    
    HFSMBuilder& withAllBasicStates() {
        std::vector<StateType> basic_states = {
            StateType::UNINITIALIZED,
            StateType::INITIALIZING,
            StateType::READY,
            StateType::OPERATIONAL,
            StateType::ERROR,
            StateType::RECOVERING,
            StateType::SHUTDOWN,
            StateType::IDLE,
            StateType::PAUSED
        };
        
        for (auto state_type : basic_states) {
            withDefaultState(state_type);
        }
        return *this;
    }
    
    HFSMBuilder& withAdvancedStates() {
        withAllBasicStates();
        // Estados adicionales
        std::vector<StateType> advanced_states = {
            StateType::CLEANING,
            StateType::TELEOP,
            StateType::AUTONOMOUS,
            StateType::DIAGNOSTICS
        };
        
        for (auto state_type : advanced_states) {
            withDefaultState(state_type);
        }
        return *this;
    }
    
    HFSMBuilder& withTransition(uint32_t from, uint32_t to, 
                               uint32_t trigger, 
                               std::function<bool()> guard = nullptr) {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        
        TransitionConfig config;
        config.from = from;
        config.to = to;
        config.trigger = trigger;
        config.guard = guard;
        config.id = "transition_" + 
                    std::to_string(from) + "_" +
                    std::to_string(to) + "_" +
                    std::to_string(trigger);
        
        transitions_.push_back(config);
        
        return *this;
    }
    
    HFSMBuilder& withTransition(StateType from, StateType to, 
                               EventType trigger, 
                               std::function<bool()> guard = nullptr) {
        return withTransition(static_cast<uint32_t>(from),
                             static_cast<uint32_t>(to),
                             static_cast<uint32_t>(trigger),
                             guard);
    }
    
    HFSMBuilder& withConfig(const HFSMConfig& config) {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        config_ = config;
        config_.syncAll();
        return *this;
    }
    
    HFSMBuilder& withTimeout(uint32_t state_ms, uint32_t global_ms, uint32_t event_ms = 100) {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        config_.state_timeout_ms = state_ms;
        config_.global_timeout_ms = global_ms;
        config_.event_processing_timeout_ms = event_ms;
        config_.syncAll();
        return *this;
    }
    
    HFSMBuilder& enableWatchdog(bool enable = true) {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        config_.enable_watchdog = enable;
        config_.syncAll();
        return *this;
    }
    
    HFSMBuilder& withLoopSleep(uint32_t ms) {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        config_.loop_sleep_ms = ms;
        return *this;
    }
    
    HFSMBuilder& withMaxConsecutiveErrors(uint32_t max_errors) {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        config_.max_consecutive_errors = max_errors;
        return *this;
    }
    
    HFSMBuilder& withShutdownTimeout(uint32_t ms) {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        config_.shutdown_timeout_ms = ms;
        return *this;
    }
    
    HFSMBuilder& withLogging(bool transitions = true, bool events = false) {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        config_.log_transitions = transitions;
        config_.log_events = events;
        return *this;
    }
    
    HFSMBuilder& withCustomState(std::unique_ptr<IState> state) {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        if (state) {
            ensureHFSMCreated();
            hfsm_->registerState(std::move(state));
        }
        return *this;
    }
    
    HFSMBuilder& withStateFactory(std::function<std::unique_ptr<IState>(uint32_t)> factory) {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        state_factory_ = std::move(factory);
        return *this;
    }
    
    HFSMBuilder& onStateEnter(uint32_t state, std::function<void()> callback) {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        state_callbacks_[state].on_enter = callback;
        return *this;
    }
    
    HFSMBuilder& onStateExit(uint32_t state, std::function<void()> callback) {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        state_callbacks_[state].on_exit = callback;
        return *this;
    }
    
    HFSMBuilder& onStateUpdate(uint32_t state, std::function<void()> callback) {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        state_callbacks_[state].on_update = callback;
        return *this;
    }
    
    HFSMBuilder& onStateEnter(StateType state, std::function<void()> callback) {
        return onStateEnter(static_cast<uint32_t>(state), callback);
    }
    
    HFSMBuilder& onStateExit(StateType state, std::function<void()> callback) {
        return onStateExit(static_cast<uint32_t>(state), callback);
    }
    
    HFSMBuilder& onStateUpdate(StateType state, std::function<void()> callback) {
        return onStateUpdate(static_cast<uint32_t>(state), callback);
    }
    
    // ------------------------------------------------------------------------
    // CONSTRUCCIÓN Y VALIDACIÓN
    // ------------------------------------------------------------------------
    std::unique_ptr<HFSMCore> build() {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        ensureHFSMCreated();
        
        if (states_registered_.empty()) {
            withEssentialStates();
        }

        applyTransitions();
        addCriticalTransitionsIfMissing();
        
        if (!validateComplete()) {
            HAL_WARN("HFSM", "HFSM construido con advertencias de validación");
        }
        
        return std::move(hfsm_);
    }
    
    HFSMCore& buildInitializeAndGetRef() {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        if (!hfsm_) {
            hfsm_ = build();
        }
        
        auto result = hfsm_->initialize();
        if (result != ErrorCode::SUCCESS) {
            throw std::runtime_error("Failed to initialize HFSM: ErrorCode " + 
                                     std::to_string(static_cast<int>(result)));
        }
        
        return *hfsm_;
    }
    
    std::unique_ptr<HFSMCore> buildAndInitialize() {
        auto hfsm = build();
        auto result = hfsm->initialize();
        if (result != ErrorCode::SUCCESS) {
            throw std::runtime_error("Failed to initialize HFSM");
        }
        return hfsm;
    }
    
    // ------------------------------------------------------------------------
    // MÉTODOS DE ACCESO E INFORMACIÓN
    // ------------------------------------------------------------------------
    const HFSMConfig& getConfig() const { 
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        return config_; 
    }
    
    HFSMConfig getConfigCopy() const { 
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        HFSMConfig copy = config_;
        copy.syncAll();
        return copy;
    }
    
    const std::vector<uint32_t>& getRegisteredStates() const { 
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        return states_registered_; 
    }
    
    bool hasState(uint32_t type) const {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        return std::find(states_registered_.begin(), states_registered_.end(), type) 
               != states_registered_.end();
    }
    
    bool hasState(StateType type) const {
        return hasState(static_cast<uint32_t>(type));
    }
    
    bool validate() const {
        std::lock_guard<std::recursive_mutex> lock(mutex_);
        if (!hfsm_) {
            return false;
        }
        
        std::vector<uint32_t> required_states = {
            static_cast<uint32_t>(StateType::UNINITIALIZED),
            static_cast<uint32_t>(StateType::SHUTDOWN)
        };
        
        for (auto required : required_states) {
            if (!hasState(required)) {
                return false;
            }
        }
        
        return true;
    }
    
    bool validateComplete() const {
        return validate() && validateTransitions();
    }

private:
    // ------------------------------------------------------------------------
    // MÉTODOS PRIVADOS
    // ------------------------------------------------------------------------
    void ensureHFSMCreated() {
        if (!hfsm_) {
            hfsm_ = std::make_unique<HFSMCore>(nullptr, config_);
        }
    }
    
    std::unique_ptr<IState> createState(uint32_t type) {
        if (state_factory_) {
            return state_factory_(type);
        }
        return StateFactory::createState(type);
    }
    
    void applyTransitions() {
        if (!hfsm_) return;
        
        for (const auto& trans : transitions_) {
            if (!hasState(trans.from) || !hasState(trans.to)) {
                HAL_WARN("HFSM", 
                    std::string("Transición ignorada - estado no registrado: ") +
                    StateFactory::stateTypeToString(trans.from) + 
                    " -> " + StateFactory::stateTypeToString(trans.to));
                continue;
            }
            
            uint32_t from_state = trans.from;
            uint32_t to_state = trans.to;
            uint32_t trigger_event = trans.trigger;
            std::function<bool()> guard_func = trans.guard;
            std::string trans_id = trans.id;
            HFSMCore* hfsm_ptr = hfsm_.get();
            
            hfsm_ptr->getEventBus().subscribe(
                trans_id,
                trigger_event,
                [hfsm_ptr, from_state, to_state, guard_func, trans_id, trigger_event](const Event& event) {
                    std::cout << "DEBUG [Transition] - Evento recibido: " 
                              << eventTypeToString(static_cast<EventType>(trigger_event))
                              << " para transición: " << trans_id << std::endl;
                    
                    if (!hfsm_ptr) {
                        std::cout << "DEBUG [Transition] - HFSM no disponible" << std::endl;
                        return;
                    }
                    
                    uint32_t current_state = hfsm_ptr->getCurrentStateType();
                    std::cout << "DEBUG [Transition] - Estado actual: " 
                              << current_state
                              << " (" << StateFactory::stateTypeToString(current_state) << ")"
                              << ", Se requiere: " << from_state
                              << " (" << StateFactory::stateTypeToString(from_state) << ")" << std::endl;
                    
                    if (current_state != from_state) {
                        std::cout << "DEBUG [Transition] - NO en estado origen requerido" << std::endl;
                        return;
                    }
                    
                    if (guard_func && !guard_func()) {
                        std::cout << "DEBUG [Transition] - Guard condition bloqueó la transición" << std::endl;
                        return;
                    }
                    
                    std::cout << "DEBUG [Transition] - Ejecutando transición: " 
                              << StateFactory::stateTypeToString(from_state) << " -> " 
                              << StateFactory::stateTypeToString(to_state) << std::endl;
                    
                    ErrorCode result = hfsm_ptr->transitionTo(to_state);
                    if (result != ErrorCode::SUCCESS) {
                        HAL_ERROR("HFSM", 
                            std::string("Falló la transición: ") +
                            StateFactory::stateTypeToString(from_state) + 
                            " -> " + StateFactory::stateTypeToString(to_state) +
                            " | Error: " + std::to_string(static_cast<int>(result)),
                            static_cast<int>(result));
                    } else {
                        HAL_INFO("HFSM", 
                            std::string("Transición exitosa: ") +
                            StateFactory::stateTypeToString(from_state) + 
                            " -> " + StateFactory::stateTypeToString(to_state));
                    }
                },
                10
            );
            
            HAL_INFO("HFSM", 
                std::string("Transición configurada: ") +
                StateFactory::stateTypeToString(trans.from) + 
                " -> " + StateFactory::stateTypeToString(trans.to) +
                " (trigger: " + eventTypeToString(static_cast<EventType>(trans.trigger)) + ")");
        }
    }
    
    void addCriticalTransitionsIfMissing() {
        if (!hfsm_) return;
        
        bool error_to_shutdown = false;
        for (const auto& trans : transitions_) {
            if (trans.from == static_cast<uint32_t>(StateType::ERROR) && 
                trans.to == static_cast<uint32_t>(StateType::SHUTDOWN)) {
                error_to_shutdown = true;
                break;
            }
        }
        
        if (hasState(StateType::ERROR) && hasState(StateType::SHUTDOWN) && !error_to_shutdown) {
            HAL_INFO("HFSM", "Añadiendo transición crítica: ERROR -> SHUTDOWN");
            
            uint32_t from = static_cast<uint32_t>(StateType::ERROR);
            uint32_t to = static_cast<uint32_t>(StateType::SHUTDOWN);
            HFSMCore* hfsm_ptr = hfsm_.get();
            
            hfsm_ptr->getEventBus().subscribe(
                "critical_error_to_shutdown",
                static_cast<uint32_t>(EventType::EMERGENCY_STOP),
                [hfsm_ptr, from, to](const Event& event) {
                    if (!hfsm_ptr) return;
                    if (hfsm_ptr->getCurrentStateType() != from) return;
                    
                    HAL_WARN("HFSM", "Shutdown de emergencia desde ERROR");
                    hfsm_ptr->transitionTo(to);
                },
                100
            );
        }
    }
    
    bool validateTransitions() const {
        bool valid = true;
        
        if (hasState(StateType::ERROR)) {
            bool error_has_exit = false;
            for (const auto& trans : transitions_) {
                if (trans.from == static_cast<uint32_t>(StateType::ERROR)) {
                    error_has_exit = true;
                    break;
                }
            }
            
            if (!error_has_exit) {
                HAL_WARN("HFSM", "Estado ERROR no tiene transiciones de salida configuradas");
            }
        }
        
        if (hasState(StateType::SHUTDOWN)) {
            bool shutdown_reachable = false;
            for (const auto& trans : transitions_) {
                if (trans.to == static_cast<uint32_t>(StateType::SHUTDOWN)) {
                    shutdown_reachable = true;
                    break;
                }
            }
            
            if (!shutdown_reachable) {
                HAL_WARN("HFSM", "Estado SHUTDOWN no es alcanzable desde ningún otro estado");
            }
        }
        
        for (const auto& trans : transitions_) {
            if (!hasState(trans.from)) {
                HAL_ERROR("HFSM", 
                    std::string("Transición inválida: estado origen no registrado - ") +
                    StateFactory::stateTypeToString(trans.from),
                    1000);
                valid = false;
            }
            
            if (!hasState(trans.to)) {
                HAL_ERROR("HFSM", 
                    std::string("Transición inválida: estado destino no registrado - ") +
                    StateFactory::stateTypeToString(trans.to),
                    1001);
                valid = false;
            }
        }
        
        return valid;
    }

    // ------------------------------------------------------------------------
    // VARIABLES MIEMBRO
    // ------------------------------------------------------------------------
    std::unique_ptr<HFSMCore> hfsm_;
    HFSMConfig config_;
    std::function<std::unique_ptr<IState>(uint32_t)> state_factory_;
    std::vector<uint32_t> states_registered_;
    std::vector<TransitionConfig> transitions_;
    std::unordered_map<uint32_t, StateCallback> state_callbacks_;
    mutable std::recursive_mutex mutex_;
};

// ----------------------------------------------------------------------------
// IMPLEMENTACIÓN DE StateFactory
// ----------------------------------------------------------------------------
inline std::unique_ptr<IState> StateFactory::createState(uint32_t type) {
    StateType state_type = static_cast<StateType>(type);
    
    switch(state_type) {
        case StateType::UNINITIALIZED:
            return std::make_unique<UninitializedState>();
        case StateType::INITIALIZING:
            return std::make_unique<InitializingState>();
        case StateType::READY:
            return std::make_unique<ReadyState>();
        case StateType::OPERATIONAL:
            return std::make_unique<OperationalState>();
        case StateType::ERROR:
            return std::make_unique<ErrorState>();
        case StateType::RECOVERING:
            return std::make_unique<RecoveringState>();
        case StateType::SHUTDOWN:
            return std::make_unique<ShutdownState>();
        case StateType::IDLE:
            return std::make_unique<IdleState>();
        case StateType::PAUSED:
            return std::make_unique<PausedState>();
        case StateType::CLEANING:
            return std::make_unique<CleaningState>();
        case StateType::DEGRADED:  
            return std::make_unique<DegradedState>();
        case StateType::NAVIGATING_TO_GOAL:
            return std::make_unique<NavigatingState>();
        case StateType::MANIPULATING:
            return std::make_unique<ManipulatingState>();
        case StateType::CHARGING:
            return std::make_unique<ChargingState>();
        case StateType::FLEET_COORDINATION:
            return std::make_unique<FleetCoordinationState>();
        case StateType::LEARNING:
            return std::make_unique<LearningState>();
        case StateType::MAPPING:
        case StateType::SLAM:
        case StateType::TELEOPERATION:
        case StateType::GRASPING:
        case StateType::PLACING:
        case StateType::VISION_PROCESSING:
        case StateType::OBJECT_TRACKING:
        case StateType::BATTERY_SWAP:
        case StateType::FORMATION_CONTROL:
        case StateType::TASK_COORDINATION:
        case StateType::ADAPTING:
        case StateType::PREDICTIVE_MAINTENANCE:
        case StateType::RECONFIGURING:
        case StateType::CALIBRATING_SENSORS:
        case StateType::PATH_PLANNING:
        case StateType::OBSTACLE_AVOIDANCE:
            return std::make_unique<BaseState>(type, stateTypeToString(type), 
                                               std::chrono::milliseconds(5000));
        default:
            return std::make_unique<BaseState>(type, "CUSTOM_STATE", 
                                               std::chrono::milliseconds(1000));
    }
}

inline std::string StateFactory::stateTypeToString(uint32_t type) {
    return ns_fsm::stateTypeToString(static_cast<StateType>(type));
}

// ----------------------------------------------------------------------------
// NAMESPACE BUILDERS - FUNCIONES FACTORY
// ----------------------------------------------------------------------------
namespace builders {
    
    inline HFSMBuilder createRoboticsHFSM() {
        HFSMBuilder builder;
        builder.forRobotics().withAllBasicStates();
        //return std::move(builder);
        return builder;
    }
    
    inline HFSMBuilder createTestHFSM() {
        HFSMBuilder builder;
        builder.forTesting().withAllBasicStates();
        //return std::move(builder);
        return builder;
    }
    
    inline HFSMBuilder createProductionHFSM() {
        HFSMBuilder builder;
        builder.forProduction().withAdvancedStates();
        //return std::move(builder);
        return builder;
    }
    
    inline HFSMBuilder createMinimalHFSM() {
        HFSMBuilder builder;
        builder.withEssentialStates();
        //return std::move(builder);
        return builder;
    }
    
    inline HFSMBuilder createCustomHFSM(const HFSMConfig& config) {
        HFSMBuilder builder;
        builder.withConfig(config).withAllBasicStates();
        //return std::move(builder);
        return builder;
    }

    inline HFSMBuilder createAdvancedRoboticsHFSM() {
        HFSMBuilder builder;
        builder.forRobotics()
               .withAllBasicStates()
               .withDefaultState(StateType::NAVIGATING_TO_GOAL)
               .withDefaultState(StateType::MANIPULATING)
               .withDefaultState(StateType::CHARGING)
               .withDefaultState(StateType::FLEET_COORDINATION)
               .withDefaultState(StateType::LEARNING)
               .withDefaultState(StateType::MAPPING)
               .withDefaultState(StateType::SLAM)
               .withTransition(StateType::READY, StateType::NAVIGATING_TO_GOAL,
                              EventType::ROBOT_NAVIGATION_START)
               .withTransition(StateType::NAVIGATING_TO_GOAL, StateType::READY,
                              EventType::ROBOT_NAVIGATION_COMPLETE)
               .withTransition(StateType::NAVIGATING_TO_GOAL, StateType::OBSTACLE_AVOIDANCE,
                              EventType::ROBOT_COLLISION_DETECTED)
               .withTransition(StateType::READY, StateType::MANIPULATING,
                              EventType::ROBOT_MANIPULATION_START)
               .withTransition(StateType::MANIPULATING, StateType::READY,
                              EventType::ROBOT_MANIPULATION_COMPLETE)
               .withTransition(StateType::READY, StateType::CHARGING,
                              EventType::ROBOT_CHARGING_START)
               .withTransition(StateType::CHARGING, StateType::READY,
                              EventType::ROBOT_CHARGING_COMPLETE)
               .withTransition(StateType::READY, StateType::FLEET_COORDINATION,
                              EventType::ROBOT_FLEET_COORDINATION)
               .withTransition(StateType::FLEET_COORDINATION, StateType::READY,
                              EventType::TASK_COMPLETE)
               .withTransition(StateType::READY, StateType::LEARNING,
                              EventType::ROBOT_LEARNING_START)
               .withTransition(StateType::LEARNING, StateType::READY,
                              EventType::ROBOT_LEARNING_COMPLETE)
               .withTransition(StateType::OPERATIONAL, StateType::MAPPING,
                              EventType::ROBOT_MAPPING_START)
               .withTransition(StateType::MAPPING, StateType::SLAM,
                              EventType::ROBOT_SLAM_ACTIVE)
               .withTransition(StateType::SLAM, StateType::READY,
                              EventType::ROBOT_MAPPING_COMPLETE);
        
        //return std::move(builder);
        return builder;
    }
    
    inline HFSMBuilder createAutonomousMobileRobotHFSM() {
        HFSMBuilder builder;
        builder.forRobotics()
               .withEssentialStates()
               .withDefaultState(StateType::NAVIGATING_TO_GOAL)
               .withDefaultState(StateType::MAPPING)
               .withDefaultState(StateType::SLAM)
               .withDefaultState(StateType::OBSTACLE_AVOIDANCE)
               .withDefaultState(StateType::CHARGING)
               .withTransition(StateType::READY, StateType::NAVIGATING_TO_GOAL,
                              EventType::NAV_GOAL_RECEIVED)
               .withTransition(StateType::NAVIGATING_TO_GOAL, StateType::READY,
                              EventType::NAV_GOAL_REACHED)
               .withTransition(StateType::NAVIGATING_TO_GOAL, StateType::OBSTACLE_AVOIDANCE,
                              EventType::NAV_OBSTACLE_DETECTED)
               .withTransition(StateType::OBSTACLE_AVOIDANCE, StateType::NAVIGATING_TO_GOAL,
                              EventType::NAV_PATH_BLOCKED)
               .withTransition(StateType::READY, StateType::MAPPING,
                              EventType::ROBOT_MAPPING_START)
               .withTransition(StateType::MAPPING, StateType::SLAM,
                              EventType::ROBOT_SLAM_ACTIVE)
               .withTransition(StateType::READY, StateType::CHARGING,
                              EventType::LOW_BATTERY)
               .onStateEnter(StateType::NAVIGATING_TO_GOAL, []() {
                   HAL_INFO("AMR", "Iniciando navegación autónoma");
               })
               .onStateEnter(StateType::OBSTACLE_AVOIDANCE, []() {
                   HAL_INFO("AMR", "Activando evasión de obstáculos");
               });
        
        //return std::move(builder);
        return builder;
    }
    
    inline HFSMBuilder createManipulationRobotHFSM() {
        HFSMBuilder builder;
        builder.forProduction()
               .withEssentialStates()
               .withDefaultState(StateType::MANIPULATING)
               .withDefaultState(StateType::GRASPING)
               .withDefaultState(StateType::PLACING)
               .withDefaultState(StateType::VISION_PROCESSING)
               .withTransition(StateType::READY, StateType::MANIPULATING,
                              EventType::ROBOT_MANIPULATION_START)
               .withTransition(StateType::MANIPULATING, StateType::GRASPING,
                              EventType::ROBOT_GRIPPER_CLOSE)
               .withTransition(StateType::GRASPING, StateType::PLACING,
                              EventType::ROBOT_ARM_MOVE_TO)
               .withTransition(StateType::PLACING, StateType::READY,
                              EventType::ROBOT_MANIPULATION_COMPLETE)
               .withTransition(StateType::READY, StateType::VISION_PROCESSING,
                              EventType::ROBOT_VISION_OBJECT_DETECTED)
               .withTransition(StateType::VISION_PROCESSING, StateType::MANIPULATING,
                              EventType::ROBOT_VISION_TRACKING_START)
               .onStateEnter(StateType::GRASPING, []() {
                   HAL_INFO("Manipulator", "Cerrando gripper sobre objeto");
               })
               .onStateEnter(StateType::PLACING, []() {
                   HAL_INFO("Manipulator", "Colocando objeto en destino");
               });
        
        //return std::move(builder);
        return builder;
    }
    
    inline HFSMBuilder createMultiRobotFleetHFSM() {
        HFSMBuilder builder;
        builder.forRobotics()
               .withEssentialStates()
               .withDefaultState(StateType::FLEET_COORDINATION)
               .withDefaultState(StateType::FORMATION_CONTROL)
               .withDefaultState(StateType::TASK_COORDINATION)
               .withDefaultState(StateType::NAVIGATING_TO_GOAL)
               .withTransition(StateType::READY, StateType::FLEET_COORDINATION,
                              EventType::ROBOT_FLEET_COORDINATION)
               .withTransition(StateType::FLEET_COORDINATION, StateType::FORMATION_CONTROL,
                              EventType::ROBOT_FORMATION_START)
               .withTransition(StateType::FORMATION_CONTROL, StateType::TASK_COORDINATION,
                              EventType::ROBOT_TASK_DELEGATION)
               .withTransition(StateType::TASK_COORDINATION, StateType::NAVIGATING_TO_GOAL,
                              EventType::ROBOT_NAVIGATION_START)
               .withTransition(StateType::NAVIGATING_TO_GOAL, StateType::READY,
                              EventType::ROBOT_NAVIGATION_COMPLETE)
               .withTransition(StateType::FORMATION_CONTROL, StateType::ERROR,
                              EventType::ROBOT_FORMATION_BREAK)
               .onStateEnter(StateType::FLEET_COORDINATION, []() {
                   HAL_INFO("Fleet", "Coordinando múltiples robots");
               })
               .onStateEnter(StateType::FORMATION_CONTROL, []() {
                   HAL_INFO("Fleet", "Manteniendo formación de robots");
               });
        
        //return std::move(builder);
        return builder;
    }

} // namespace builders

} // namespace ns_fsm

#endif // HFSM_BUILDER_FIXED_HPP
