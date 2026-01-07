/**
 * @file     hfsm_config_manager.hpp
 * @brief    Sistema avanzado de gestión de configuración para HFSM
 * @author   Lucas C.
 * @date     27/12/2025
 * @version  1.0
 * @license  MIT
 * 
 * @details  Este archivo define el sistema completo de gestión de configuración,
 *           incluyendo:
 *           - Clase ConfigManager con soporte para múltiples formatos (JSON, XML, YAML, INI)
 *           - Sistema de prioridades para resolución de conflictos
 *           - Validación con JSON Schema
 *           - Callbacks para cambios dinámicos de configuración
 *           - Historial de cambios y auditoría
 *           - Plantillas para acceso seguro a datos tipados
 *           - Configuraciones específicas para HFSM
 *           - Excepciones especializadas para manejo de errores
 * 
 * @note     Usa shared_mutex para acceso concurrente seguro. Incluye sistema de
 *           monitoreo automático de archivos para recarga en caliente.
 * 
 * @warning  La implementación de parsing de XML, YAML e INI requiere librerías
 *           externas. La validación de schema es básica en esta versión.
 * 
 * @see      hfsm_config_core.hpp - Estructura principal de configuración HFSM
 */
#ifndef HFSM_CONFIG_MANAGER_HPP
#define HFSM_CONFIG_MANAGER_HPP

#include "hfsm/hfsm_config_core.hpp"
#include <filesystem>
#include <memory>
#include <functional>
#include <map>
#include <set>
#include <atomic>
#include <shared_mutex>
#include <thread>
#include <optional>
#include <mutex>


namespace ns_fsm {

namespace fs = std::filesystem;

// ============================================================================
// ENUMS Y ESTRUCTURAS BÁSICAS
// ============================================================================

enum class ConfigFormat {
    JSON,
    XML,
    YAML,
    INI
};

enum class ConfigSource {
    DEFAULT,
    FILE,
    DATABASE,
    NETWORK,
    RUNTIME
};

enum class ConfigPriority : uint8_t {
    LOWEST = 0,
    LOW = 25,
    NORMAL = 50,
    HIGH = 75,
    HIGHEST = 100,
    CRITICAL = 127
};

struct ConfigMetadata {
    std::string version;
    std::string description;
    std::string author;
    std::chrono::system_clock::time_point created;
    std::chrono::system_clock::time_point modified;
    std::string checksum;
    ConfigSource source;
    ConfigPriority priority;
    std::map<std::string, std::string> tags;
    
    ConfigMetadata() 
        : version("1.0.0")
        , source(ConfigSource::DEFAULT)
        , priority(ConfigPriority::NORMAL) {}
    
    nlohmann::json toJson() const;
    static ConfigMetadata fromJson(const nlohmann::json& j);
};

struct ConfigChange {
    std::string path;
    std::string old_value;
    std::string new_value;
    std::chrono::system_clock::time_point timestamp;
    std::string user;
    std::string reason;
    
    nlohmann::json toJson() const;
};

// ============================================================================
// EXCEPCIONES ESPECÍFICAS DE CONFIGURACIÓN
// ============================================================================

class ConfigException : public std::runtime_error {
public:
    ConfigException(const std::string& msg, const std::string& path = "")
        : std::runtime_error(msg), config_path_(path) {}
    
    const std::string& getConfigPath() const { return config_path_; }
    
private:
    std::string config_path_;
};

class ConfigValidationException : public ConfigException {
public:
    ConfigValidationException(const std::string& msg, 
                            const std::string& path = "",
                            const std::string& schema_path = "")
        : ConfigException(msg, path), schema_path_(schema_path) {}
    
    const std::string& getSchemaPath() const { return schema_path_; }
    
private:
    std::string schema_path_;
};

class ConfigParseException : public ConfigException {
public:
    ConfigParseException(const std::string& msg, 
                        const std::string& path = "",
                        int line = -1, int column = -1)
        : ConfigException(msg, path), line_(line), column_(column) {}
    
    int getLine() const { return line_; }
    int getColumn() const { return column_; }
    
private:
    int line_;
    int column_;
};

// ============================================================================
// MANEJADOR DE CONFIGURACIÓN PRINCIPAL
// ============================================================================

class ConfigManager {
private:
    // Tipos internos
    struct ConfigEntry {
        std::string key;
        nlohmann::json data;
        ConfigMetadata metadata;
        std::chrono::steady_clock::time_point loaded_at;
        bool validated;
        std::string schema_id;
        
        ConfigEntry() : validated(false) {}
    };
    
    // Callback para cambios
    using ConfigChangeCallback = std::function<void(
        const std::string& key,
        const nlohmann::json& old_value,
        const nlohmann::json& new_value,
        const ConfigMetadata& metadata
    )>;
    
    // Datos miembro
    std::map<std::string, ConfigEntry> config_store_;
    std::map<std::string, nlohmann::json> schemas_;
    std::map<std::string, std::set<ConfigChangeCallback>> change_callbacks_;
    std::vector<ConfigChange> change_history_;
    
    // Control de concurrencia
    mutable std::shared_mutex store_mutex_;
    mutable std::shared_mutex schema_mutex_;
    mutable std::mutex callback_mutex_;
    
    // Configuración del manager
    fs::path config_base_path_;
    fs::path schema_base_path_;
    ConfigFormat default_format_;
    size_t max_change_history_{1000};
    bool auto_reload_{false};
    std::atomic<bool> running_{false};
    std::thread file_watcher_thread_;
    std::map<fs::path, std::chrono::steady_clock::time_point> watched_files_;
    
    // Métodos privados
    void validateAgainstSchema(const nlohmann::json& data, 
                               const std::string& schema_id);
    void notifyChangeCallbacks(const std::string& key,
                               const nlohmann::json& old_value,
                               const nlohmann::json& new_value,
                               const ConfigMetadata& metadata);
    void fileWatcherLoop();
    void loadSchema(const fs::path& schema_path);
    void loadAllSchemas();
    nlohmann::json mergeConfigs(const nlohmann::json& base, 
                                const nlohmann::json& override);
    ConfigFormat detectFormat(const fs::path& file_path);
    nlohmann::json parseFile(const fs::path& file_path);
    void saveToFile(const fs::path& file_path, 
                    const nlohmann::json& data, 
                    ConfigFormat format);
    void backupConfig(const std::string& key, const nlohmann::json& old_data);
    
public:
    // ------------------------------------------------------------------------
    // CONSTRUCTOR Y DESTRUCTOR
    // ------------------------------------------------------------------------
    ConfigManager(const fs::path& base_path = "config",
                  ConfigFormat default_format = ConfigFormat::JSON);
    ~ConfigManager();
    
    // No copiable
    ConfigManager(const ConfigManager&) = delete;
    ConfigManager& operator=(const ConfigManager&) = delete;
    
    // Movible
    ConfigManager(ConfigManager&&) = default;
    ConfigManager& operator=(ConfigManager&&) = default;
    
    // ------------------------------------------------------------------------
    // CONFIGURACIÓN DEL MANAGER
    // ------------------------------------------------------------------------
    void setBasePath(const fs::path& path);
    void setDefaultFormat(ConfigFormat format);
    void enableAutoReload(bool enable);
    void setMaxHistorySize(size_t max_size);
    
    // ------------------------------------------------------------------------
    // CARGA Y GUARDADO DE CONFIGURACIÓN
    // ------------------------------------------------------------------------
    void loadFromFile(const std::string& key, 
                      const fs::path& file_path,
                      const std::string& schema_id = "",
                      ConfigPriority priority = ConfigPriority::NORMAL);
    
    void loadFromString(const std::string& key,
                        const std::string& content,
                        ConfigFormat format,
                        const std::string& schema_id = "",
                        ConfigPriority priority = ConfigPriority::NORMAL);
    
    void loadFromJson(const std::string& key,
                      const nlohmann::json& data,
                      const std::string& schema_id = "",
                      ConfigPriority priority = ConfigPriority::NORMAL,
                      const ConfigMetadata& metadata = ConfigMetadata());
    
    void saveToFile(const std::string& key,
                    const fs::path& file_path,
                    ConfigFormat format = ConfigFormat::JSON);
    
    // ------------------------------------------------------------------------
    // GESTIÓN DE SCHEMAS
    // ------------------------------------------------------------------------
    void registerSchema(const std::string& schema_id,
                        const nlohmann::json& schema);
    
    void loadSchemaFromFile(const std::string& schema_id,
                            const fs::path& schema_path);
    
    bool validate(const std::string& key, 
                  const nlohmann::json& data,
                  const std::string& schema_id = "");
    
    // ------------------------------------------------------------------------
    // ACCESO A CONFIGURACIÓN
    // ------------------------------------------------------------------------
    template<typename T>
    T get(const std::string& key, const std::string& path = "", 
          const T& default_value = T()) const;
    
    template<typename T>
    std::optional<T> getOptional(const std::string& key, 
                                 const std::string& path = "") const;
    
    nlohmann::json getJson(const std::string& key, 
                          const std::string& path = "") const;
    
    bool exists(const std::string& key) const;
    std::vector<std::string> listConfigs() const;
    
    // ------------------------------------------------------------------------
    // MODIFICACIÓN DE CONFIGURACIÓN
    // ------------------------------------------------------------------------
    template<typename T>
    void set(const std::string& key, const std::string& path,
             const T& value, const std::string& user = "system",
             const std::string& reason = "manual update");
    
    void setJson(const std::string& key, const std::string& path,
                 const nlohmann::json& value, const std::string& user = "system",
                 const std::string& reason = "manual update");
    
    void remove(const std::string& key, const std::string& path,
                const std::string& user = "system",
                const std::string& reason = "removal");
    
    // ------------------------------------------------------------------------
    // SUSCRIPCIONES A CAMBIOS
    // ------------------------------------------------------------------------
    void subscribeToChanges(const std::string& key,
                           ConfigChangeCallback callback);
    
    void unsubscribeFromChanges(const std::string& key,
                               ConfigChangeCallback callback);
    
    // ------------------------------------------------------------------------
    // HISTORIAL Y METADATOS
    // ------------------------------------------------------------------------
    std::vector<ConfigChange> getChangeHistory(const std::string& key = "",
                                              size_t limit = 100) const;
    
    ConfigMetadata getMetadata(const std::string& key) const;
    void updateMetadata(const std::string& key,
                       const ConfigMetadata& metadata);
    
    // ------------------------------------------------------------------------
    // UTILIDADES
    // ------------------------------------------------------------------------
    nlohmann::json getMergedConfig(const std::vector<std::string>& keys) const;
    void reloadFromSource(const std::string& key);
    void exportAll(const fs::path& export_dir, 
                   ConfigFormat format = ConfigFormat::JSON) const;
    
    // ------------------------------------------------------------------------
    // CONFIGURACIONES ESPECÍFICAS DEL HFSM
    // ------------------------------------------------------------------------
    HFSMConfig getHFSMConfig(const std::string& config_key = "hfsm") const;
    void updateHFSMConfig(const HFSMConfig& config,
                         const std::string& config_key = "hfsm",
                         const std::string& user = "system");
    
    // ------------------------------------------------------------------------
    // ESTADÍSTICAS
    // ------------------------------------------------------------------------
    nlohmann::json getStatistics() const;
    
    // ------------------------------------------------------------------------
    // MÉTODOS ESTÁTICOS DE UTILIDAD
    // ------------------------------------------------------------------------
    static std::string formatToString(ConfigFormat format);
    static ConfigFormat stringToFormat(const std::string& str);
    static std::string generateChecksum(const nlohmann::json& data);
    static nlohmann::json createDefaultHFSMConfig();
    static nlohmann::json createDefaultAlertConfig();
};

// ============================================================================
// TEMPLATE IMPLEMENTATIONS (in header for linker)
// ============================================================================

template<typename T>
T ConfigManager::get(const std::string& key, const std::string& path,
                     const T& default_value) const {
    std::shared_lock<std::shared_mutex> lock(store_mutex_);
    
    auto it = config_store_.find(key);
    if (it == config_store_.end()) {
        return default_value;
    }
    
    if (path.empty()) {
        return it->second.data.get<T>();
    }
    
    // Usar json pointer para navegar
    try {
        nlohmann::json::json_pointer ptr(path);
        return it->second.data.at(ptr).get<T>();
    } catch (...) {
        return default_value;
    }
}

template<typename T>
std::optional<T> ConfigManager::getOptional(const std::string& key,
                                           const std::string& path) const {
    std::shared_lock<std::shared_mutex> lock(store_mutex_);
    
    auto it = config_store_.find(key);
    if (it == config_store_.end()) {
        return std::nullopt;
    }
    
    try {
        if (path.empty()) {
            return it->second.data.get<T>();
        } else {
            nlohmann::json::json_pointer ptr(path);
            return it->second.data.at(ptr).get<T>();
        }
    } catch (...) {
        return std::nullopt;
    }
}

template<typename T>
void ConfigManager::set(const std::string& key, const std::string& path,
                        const T& value, const std::string& user,
                        const std::string& reason) {
    std::unique_lock<std::shared_mutex> lock(store_mutex_);
    
    auto it = config_store_.find(key);
    if (it == config_store_.end()) {
        throw ConfigException("Config key not found: " + key);
    }
    
    // Crear copia del valor anterior
    nlohmann::json old_data = it->second.data;
    nlohmann::json new_data = old_data;
    
    try {
        if (path.empty()) {
            new_data = value;
        } else {
            nlohmann::json::json_pointer ptr(path);
            new_data[ptr] = value;
        }
        
        // Validar si hay schema
        if (!it->second.schema_id.empty()) {
            validateAgainstSchema(new_data, it->second.schema_id);
        }
        
        // Actualizar
        it->second.data = new_data;
        it->second.metadata.modified = std::chrono::system_clock::now();
        
        // Registrar cambio
        ConfigChange change;
        change.path = key + (path.empty() ? "" : ":" + path);
        change.old_value = old_data.dump();
        change.new_value = new_data.dump();
        change.timestamp = std::chrono::system_clock::now();
        change.user = user;
        change.reason = reason;
        
        change_history_.push_back(change);
        if (change_history_.size() > max_change_history_) {
            change_history_.erase(change_history_.begin());
        }
        
        // Notificar callbacks
        lock.unlock();  // Liberar antes de notificar
        notifyChangeCallbacks(key, old_data, new_data, it->second.metadata);
        
        // Backup
        backupConfig(key, old_data);
        
    } catch (const std::exception& e) {
        throw ConfigException(std::string("Failed to set config: ") + e.what(), key);
    }
}

} // namespace ns_fsm

#endif // HFSM_CONFIG_MANAGER_HPP
