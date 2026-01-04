// hfsm_core.hpp - VERSIÓN CORREGIDA SIN DEPENDENCIAS CIRCULARES
#ifndef HFSM_CORE_FIXED_HPP
#define HFSM_CORE_FIXED_HPP

#include "hfsm_config_core.hpp"
#include "hfsm_error_code.hpp"

#include <memory>
#include <unordered_map>
#include <queue>
#include <mutex>
#include <atomic>
#include <thread>
#include <condition_variable>
#include <chrono>
#include <functional>
#include <string>
#include <stdexcept>

namespace ns_fsm {

// ============================================================================
// FORWARD DECLARATIONS (romper dependencias circulares)
// ============================================================================

class IState;
struct Event;
class ModuleManager;

// ============================================================================
// EVENT BUS INDEPENDIENTE
// ============================================================================

class EventBus {
private:
    struct Subscription {
        std::string id;
        std::function<void(const Event&)> callback;
        int priority{0};
    };
    
    std::unordered_map<uint32_t, std::vector<Subscription>> subscribers_;
    std::queue<Event> event_queue_;
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::atomic<uint64_t> event_counter_{0};
    
public:
    EventBus() = default;
    
    // No copiable
    EventBus(const EventBus&) = delete;
    EventBus& operator=(const EventBus&) = delete;
    
    // Movible
    EventBus(EventBus&&) = default;
    EventBus& operator=(EventBus&&) = default;
    
    // Métodos públicos (implementación en .cpp)
    void subscribe(const std::string& id, uint32_t event_type, 
                   std::function<void(const Event&)> callback, int priority = 0);
    
    void unsubscribe(const std::string& id, uint32_t event_type);
    
    void publish(const Event& event);
    
    bool processOne();
    
    struct Stats {
        size_t pending_events{0};
        size_t total_subscribers{0};
        size_t event_types{0};
    };
    
    Stats getStats() const;
    
    void clear();
};

// ============================================================================
// INTERFAZ DE ESTADO BÁSICA
// ============================================================================

class IState {
public:
    virtual ~IState() = default;
    
    // Identificación
    virtual uint32_t getType() const = 0;
    virtual std::string getName() const = 0;
    
    // Contexto
    virtual void setHFSMContext(void* context) = 0;
    
    // Ciclo de vida
    virtual bool onEnter() = 0;
    virtual bool onExit() = 0;
    virtual void update() = 0;
    
    // Manejo de eventos
    virtual bool handleEvent(const Event& event) = 0;
    
    // Timeout y validación
    virtual std::chrono::milliseconds getTimeout() const = 0;
    virtual bool isFinalState() const = 0;
    
    // Validación de transiciones
    virtual bool canTransitionTo(uint32_t target) const = 0;
};

// ============================================================================
// HFSM CORE - DECLARACIÓN PRINCIPAL
// ============================================================================

class HFSMCore {
private:
    // Estados registrados
    std::unordered_map<uint32_t, std::unique_ptr<IState>> states_;
    IState* current_state_{nullptr};
    
    // Sistema de eventos
    EventBus event_bus_;
    
    // Gestión de módulos
    ModuleManager* module_manager_{nullptr};
    
    // Hilo principal
    std::thread main_thread_;
    mutable std::recursive_mutex mutex_;
    std::atomic<bool> running_{false};
    std::atomic<bool> shutdown_requested_{false};
    std::atomic<bool> initialized_{false};
    
    // Temporizadores
    std::chrono::steady_clock::time_point last_state_change_;
    std::chrono::steady_clock::time_point last_update_;
    
    // Configuración
    HFSMConfig config_;
    
    // Estadísticas
    struct {
        uint64_t total_loops{0};
        uint64_t events_processed{0};
        uint64_t state_transitions{0};
        uint64_t errors{0};
    } stats_;
    
    // Métodos privados (implementación en .cpp)
    void runLoop();
    void checkStateTimeout();
    void emergencyShutdown();
    void logStatistics();
    ErrorCode transitionToInternal(uint32_t target, bool emergency = false);
    void gracefulShutdown();
    
public:
    // Constructor y destructor
    HFSMCore(ModuleManager* module_manager = nullptr, 
             const HFSMConfig& config = ns_fsm::HFSMConfig::Default());
    ~HFSMCore();
    
    // No copiable
    HFSMCore(const HFSMCore&) = delete;
    HFSMCore& operator=(const HFSMCore&) = delete;
    
    // Movible
    HFSMCore(HFSMCore&&) = default;
    HFSMCore& operator=(HFSMCore&&) = default;
    
    // Métodos públicos
    ErrorCode initialize();
    ErrorCode shutdown(bool emergency = false);
    ErrorCode reset();
    ErrorCode registerState(std::unique_ptr<IState> state);
    ErrorCode transitionTo(uint32_t target);
    void publishEvent(const Event& event);  // <-- AÑADIDO
    
    // Métodos de acceso
    uint32_t getCurrentStateType() const;
    std::string getCurrentStateName() const;
    bool isRunning() const;
    bool isInitialized() const;
    
    EventBus& getEventBus();
    ModuleManager* getModuleManager() const;
    void setModuleManager(ModuleManager* manager);
    HFSMConfig getConfig() const;
    ErrorCode updateConfig(const HFSMConfig& new_config);
    
    nlohmann::json getStatistics() const;

   // Métodos de validación de estados
    bool isStateRegistered(uint32_t state_type) const;
    std::vector<uint32_t> getRegisteredStates() const;
    
    // Métodos para validar transiciones alcanzables
    bool isStateTransitionPossible(uint32_t from_state, uint32_t to_state) const;
    bool isStateReachableFromCurrent(uint32_t target_state) const;
};

} // namespace ns_fsm

#endif // HFSM_CORE_FIXED_HPP
