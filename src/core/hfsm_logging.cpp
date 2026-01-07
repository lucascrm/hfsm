/**
 * @file     hfsm_logging.cpp
 * @brief    Implementación del sistema de logging estructurado para HFSM
 * @author   Lucas C.
 * @date     27/12/2025
 * @version  1.0
 * @license  MIT
 * 
 * @details  Este archivo implementa el sistema completo de logging estructurado,
 *           incluyendo:
 *           - Logging síncrono para inicialización y casos críticos
 *           - Logging asíncrono con buffer para mejor rendimiento
 *           - Rotación automática de archivos de log
 *           - Formateo de timestamps y metadatos
 *           - Filtrado por nivel y subsistema
 *           - Métodos específicos para dominio HFSM (transiciones, eventos, errores)
 *           - Gestión de hilos y sincronización segura
 * 
 * @note     Usa un patrón producer-consumer con buffer para logging asíncrono.
 *           Incluye rotación automática basada en tamaño y número máximo de archivos.
 *           Es compatible con el sistema de diagnóstico del hardware.
 * 
 * @warning  El logging síncrono (logDirect) puede afectar rendimiento si se usa
 *           en bucles rápidos. Preferir el logging asíncrono para operaciones normales.
 * 
 * @see      hfsm_logging.hpp - Declaración de clases y configuración
 */
#include "hfsm_logging.hpp"

#include <iomanip>
#include <sstream>
#include <filesystem>
#include <algorithm>
#include <unistd.h>
#include <fstream>
#include <iostream>

namespace ns_fsm {

namespace fs = std::filesystem;

// ============================================================================
// FUNCIONES AUXILIARES
// ============================================================================

std::string formatTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::tm tm;
    localtime_r(&time_t, &tm);
    
    std::ostringstream timestamp;
    timestamp << std::put_time(&tm, "%Y-%m-%dT%H:%M:%S") 
              << '.' << std::setfill('0') << std::setw(3) << ms.count();
    return timestamp.str();
}

// ============================================================================
// IMPLEMENTACIÓN DE HFSMLOGGER
// ============================================================================

HFSMLogger::HFSMLogger() 
    : initialized_(false)
    , running_(false)
    , current_file_size_(0) {
    // Constructor vacío
}

HFSMLogger::~HFSMLogger() {
    shutdown();
}

HFSMLogger& HFSMLogger::getInstance() {
    static HFSMLogger instance;
    return instance;
}

void HFSMLogger::logDirect(const std::string& subsystem,
                          hardware::DiagnosticLevel level,
                          const std::string& message,
                          const nlohmann::json& context) {
    // Versión síncrona para inicialización
    
    // Construir objeto JSON
    nlohmann::json log_entry = {
        {"timestamp", formatTimestamp()},
        {"level", hardware::diagnosticsLevelToString(level)},
        {"subsystem", subsystem},
        {"message", message}
    };
    
    // Añadir thread ID
    std::ostringstream thread_id_ss;
    thread_id_ss << std::this_thread::get_id();
    log_entry["thread_id"] = thread_id_ss.str();
    
    // Añadir process ID
    log_entry["process_id"] = static_cast<int>(getpid());
    
    // Añadir contexto si existe
    if (!context.empty()) {
        log_entry["context"] = context;
    }
    
    std::string log_str = log_entry.dump();
    
    // Escribir directamente si file logging está habilitado
    if (config_.enable_file_logging && log_file_.is_open()) {
        try {
            // Verificar rotación
            if (current_file_size_ + log_str.size() + 1 > 
                config_.max_log_size_mb * 1024 * 1024) {
                rotateLogFile();
            }
            
            if (log_file_.is_open()) {
                log_file_ << log_str << std::endl;
                log_file_.flush();
                current_file_size_ += log_str.size() + 1;
            }
        } catch (const std::exception& e) {
            // Silenciar errores durante inicialización
        }
    }
    
    // También mostrar en consola si está habilitado
    if (config_.enable_console_logging) {
        std::string level_str = hardware::diagnosticsLevelToString(level);
        std::cout << "[" << level_str << "][" << subsystem << "] " 
                  << message << std::endl;
    }
}

void HFSMLogger::initialize(const LogConfig& config) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (initialized_) {
        return;
    }
    
    // Asignar configuración
    config_ = config;
    
    try {
        // 1. Configurar file logging primero
        if (config_.enable_file_logging) {
            setupFileLogging();
        }
        
        // 2. Log de inicialización (síncrono)
        logDirect("HFSMLogger", hardware::DiagnosticLevel::INFO,
                 "Logger initialized",
                 {
                    {"file_logging", config_.enable_file_logging},
                    {"max_size_mb", config_.max_log_size_mb},
                    {"max_files", config_.max_log_files}
                 });
        
        // 3. Iniciar thread flush SOLO si structured logging está habilitado
        if (config_.enable_structured_logging) {
            startFlushThread();
        }
        
        // 4. Marcar como inicializado
        initialized_ = true;
        
    } catch (const std::exception& e) {
        initialized_ = false;
        running_ = false;
        
        // Intentar limpiar
        if (flush_thread_.joinable()) {
            flush_thread_.join();
        }
        if (log_file_.is_open()) {
            log_file_.close();
        }
        throw;
    }
}

void HFSMLogger::shutdown() {
    if (!initialized_) {
        return;
    }
    
    // 1. Detener el thread worker
    running_ = false;
    buffer_not_empty_.notify_all();
    
    // 2. Unir el thread si es joinable
    if (flush_thread_.joinable()) {
        flush_thread_.join();
    }
    
    // 3. Vaciar buffer final al archivo (síncrono)
    flushToFile();
    
    // 4. Cerrar archivo
    if (log_file_.is_open()) {
        try {
            log_file_.close();
        } catch (...) {
            // Ignorar errores al cerrar
        }
    }
    
    initialized_ = false;
}

void HFSMLogger::logStructured(const std::string& subsystem,
                              hardware::DiagnosticLevel level,
                              const std::string& message,
                              const nlohmann::json& context) {
    
    // Verificar si estamos inicializados
    if (!initialized_.load()) {
        // Si no estamos inicializados, usar logging directo
        logDirect(subsystem, level, message, context);
        return;
    }
    
    // Verificar si debemos registrar este log
    if (!shouldLog(subsystem, level)) {
        return;
    }
    
    // Construir objeto JSON
    nlohmann::json log_entry = {
        {"timestamp", formatTimestamp()},
        {"level", hardware::diagnosticsLevelToString(level)},
        {"subsystem", subsystem},
        {"message", message}
    };
    
    // Añadir thread ID
    std::ostringstream thread_id_ss;
    thread_id_ss << std::this_thread::get_id();
    log_entry["thread_id"] = thread_id_ss.str();
    
    // Añadir process ID
    log_entry["process_id"] = static_cast<int>(getpid());
    
    // Añadir contexto si existe
    if (!context.empty()) {
        log_entry["context"] = context;
    }
    
    // Añadir a buffer y notificar al thread
    {
        std::lock_guard<std::mutex> lock(mutex_);
        log_buffer_.push(log_entry.dump());
        buffer_not_empty_.notify_one();
    }
    
    // También mostrar en consola si está habilitado
    if (config_.enable_console_logging && level >= hardware::DiagnosticLevel::INFO) {
        std::cout << "[" << hardware::diagnosticsLevelToString(level) << "][" 
                  << subsystem << "] " << message << std::endl;
    }
}

void HFSMLogger::setupFileLogging() {
    // Crear directorio de logs si no existe
    try {
        fs::create_directories("logs");
    } catch (const fs::filesystem_error&) {
        config_.enable_file_logging = false;
        return;
    }
    
    // Buscar archivo más reciente o crear uno nuevo
    for (size_t i = 0; i < config_.max_log_files; i++) {
        std::string file = "logs/hfsm_" + std::to_string(i) + ".json";
        if (fs::exists(file)) {
            log_file_path_ = file;
            try {
                current_file_size_ = fs::file_size(file);
            } catch (...) {
                current_file_size_ = 0;
            }
            break;
        }
    }
    
    // Si no se encontró archivo existente, crear uno nuevo
    if (log_file_path_.empty()) {
        log_file_path_ = "logs/hfsm_0.json";
        current_file_size_ = 0;
    }
    
    // Abrir archivo en modo append
    try {
        log_file_.open(log_file_path_, std::ios::app);
        if (!log_file_.is_open()) {
            config_.enable_file_logging = false;
        }
    } catch (const std::exception&) {
        config_.enable_file_logging = false;
    }
}

void HFSMLogger::startFlushThread() {
    if (running_.load()) {
        return;
    }
    
    running_ = true;
    flush_thread_ = std::thread([this]() {
        flushWorker();
    });
    
    // Pequeña pausa para asegurar que el thread se inicie
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

void HFSMLogger::flushWorker() {
    while (running_.load()) {
        std::queue<std::string> buffer_copy;
        
        {
            // Esperar hasta que haya datos o se solicite parada
            std::unique_lock<std::mutex> lock(mutex_);
            
            buffer_not_empty_.wait_for(lock, 
                std::chrono::milliseconds(100),
                [this]() { 
                    return !log_buffer_.empty() || !running_.load(); 
                });
            
            // Verificar si debemos salir
            if (!running_.load()) {
                break;
            }
            
            // Si no hay datos, continuar
            if (log_buffer_.empty()) {
                continue;
            }
            
            // Mover los datos
            buffer_copy.swap(log_buffer_);
        }
        
        // Escribir al archivo
        if (!log_file_.is_open() || !config_.enable_file_logging) {
            continue;
        }
        
        try {
            while (!buffer_copy.empty()) {
                std::string log_entry = buffer_copy.front();
                buffer_copy.pop();
                
                // Verificar rotación
                if (current_file_size_ + log_entry.size() + 1 > 
                    config_.max_log_size_mb * 1024 * 1024) {
                    rotateLogFile();
                    if (!log_file_.is_open()) {
                        break;
                    }
                }
                
                // Escribir entrada
                log_file_ << log_entry << std::endl;
                log_file_.flush();
                
                // Actualizar tamaño
                current_file_size_ += log_entry.size() + 1;
            }
        } catch (const std::exception&) {
            // Continuar incluso si hay errores de escritura
        }
    }
    
    // Limpiar buffer final
    flushToFile();
}

void HFSMLogger::flushToFile() {
    if (!config_.enable_file_logging || !log_file_.is_open()) {
        return;
    }
    
    std::queue<std::string> buffer_copy;
    
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (log_buffer_.empty()) {
            return;
        }
        buffer_copy.swap(log_buffer_);
    }
    
    try {
        while (!buffer_copy.empty()) {
            std::string log_entry = buffer_copy.front();
            buffer_copy.pop();
            
            // Verificar rotación
            if (current_file_size_ + log_entry.size() + 1 > 
                config_.max_log_size_mb * 1024 * 1024) {
                rotateLogFile();
                if (!log_file_.is_open()) {
                    return;
                }
            }
            
            // Escribir
            log_file_ << log_entry << std::endl;
            log_file_.flush();
            current_file_size_ += log_entry.size() + 1;
        }
    } catch (const std::exception&) {
        // Ignorar errores en el flush final
    }
}

void HFSMLogger::rotateLogFile() {
    if (!log_file_.is_open()) {
        return;
    }
    
    try {
        // Cerrar archivo actual
        log_file_.close();
        
        // Rotar archivos existentes
        for (int i = static_cast<int>(config_.max_log_files) - 1; i > 0; i--) {
            std::string old_file = "logs/hfsm_" + std::to_string(i - 1) + ".json";
            std::string new_file = "logs/hfsm_" + std::to_string(i) + ".json";
            
            if (fs::exists(old_file)) {
                if (fs::exists(new_file)) {
                    fs::remove(new_file);
                }
                fs::rename(old_file, new_file);
            }
        }
        
        // Crear nuevo archivo actual
        log_file_path_ = "logs/hfsm_0.json";
        current_file_size_ = 0;
        
        // Limpiar archivos viejos
        for (size_t i = config_.max_log_files; i < config_.max_log_files + 5; i++) {
            std::string old_file = "logs/hfsm_" + std::to_string(i) + ".json";
            if (fs::exists(old_file)) {
                fs::remove(old_file);
            }
        }
        
        // Abrir nuevo archivo
        log_file_.open(log_file_path_, std::ios::app);
        
        if (!log_file_.is_open()) {
            config_.enable_file_logging = false;
        }
        
    } catch (const std::exception&) {
        config_.enable_file_logging = false;
    }
}

bool HFSMLogger::shouldLog(const std::string& subsystem, 
                          hardware::DiagnosticLevel level) const {
    // Filtrar por nivel
    if (level < config_.min_level) {
        return false;
    }
    
    // Filtrar por subsistema
    if (!config_.filtered_subsystems.empty()) {
        if (std::find(config_.filtered_subsystems.begin(),
                     config_.filtered_subsystems.end(),
                     subsystem) != config_.filtered_subsystems.end()) {
            return false;
        }
    }
    
    return true;
}

// Métodos específicos de dominio
void HFSMLogger::logStateTransition(uint32_t from_state, uint32_t to_state,
                                   const std::string& trigger,
                                   bool success) {
    logStructured("HFSM",
                 success ? hardware::DiagnosticLevel::INFO : hardware::DiagnosticLevel::WARN,
                 "State transition " + std::string(success ? "successful" : "failed"),
                 {
                    {"from_state", from_state},
                    {"to_state", to_state},
                    {"trigger", trigger},
                    {"success", success}
                 });
}

void HFSMLogger::logEventProcessed(const Event& event, bool success,
                                  const std::string& handler) {
    logStructured("EventSystem",
                 success ? hardware::DiagnosticLevel::DEBUG : hardware::DiagnosticLevel::ERROR,
                 "Event processed",
                 {
                    {"event_type", static_cast<uint32_t>(event.type)},
                    {"handler", handler},
                    {"success", success},
                    {"source", event.source_module}
                 });
}

void HFSMLogger::logError(ErrorCode code, const std::string& context,
                         const std::string& component) {
    logStructured(component,
                 hardware::DiagnosticLevel::ERROR,
                 "Error occurred",
                 {
                    {"error_code", static_cast<int32_t>(code)},
                    {"context", context}
                 });
}

void HFSMLogger::logPerformanceMetric(const std::string& metric_name,
                                     double value,
                                     const std::string& unit) {
    logStructured("Performance",
                 hardware::DiagnosticLevel::DEBUG,
                 "Performance metric",
                 {
                    {"metric", metric_name},
                    {"value", value},
                    {"unit", unit}
                 });
}

void HFSMLogger::setMinLogLevel(hardware::DiagnosticLevel level) {
    std::lock_guard<std::mutex> lock(mutex_);
    config_.min_level = level;
}

nlohmann::json HFSMLogger::getStatistics() const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    nlohmann::json stats;
    stats["initialized"] = initialized_.load();
    stats["running"] = running_.load();
    stats["file_logging_enabled"] = config_.enable_file_logging;
    stats["current_file"] = log_file_path_;
    stats["current_file_size"] = current_file_size_;
    stats["pending_entries"] = log_buffer_.size();
    
    return stats;
}

} // namespace ns_fsm
// file content end
