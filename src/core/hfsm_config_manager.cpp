// hfsm_config_manager.cpp
#include "hfsm_config_manager.hpp"
#include "hfsm_logging.hpp"

#include <fstream>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <openssl/sha.h>  // Para checksums

namespace ns_fsm {

// ============================================================================
// IMPLEMENTACIÓN DE CONFIGMANAGER
// ============================================================================

ConfigManager::ConfigManager(const fs::path& base_path,
                             ConfigFormat default_format)
    : config_base_path_(fs::absolute(base_path))
    , schema_base_path_(config_base_path_ / "schemas")
    , default_format_(default_format)
    , auto_reload_(false) {
    
    // Crear estructura de directorios si no existe
    fs::create_directories(config_base_path_);
    fs::create_directories(schema_base_path_);
    fs::create_directories(config_base_path_ / "defaults");
    fs::create_directories(config_base_path_ / "overrides");
    fs::create_directories(config_base_path_ / "backup");
    
    // Cargar schemas por defecto
    loadAllSchemas();
    
    HAL_INFO("ConfigManager", "Config manager inicializado",
        {
            {"base_path", config_base_path_.string()},
            {"default_format", formatToString(default_format_)}
        });
}

ConfigManager::~ConfigManager() {
    running_ = false;
    if (file_watcher_thread_.joinable()) {
        file_watcher_thread_.join();
    }
    
    HAL_DEBUG("ConfigManager", "Config manager destruido");
}

void ConfigManager::loadFromFile(const std::string& key,
                                const fs::path& file_path,
                                const std::string& schema_id,
                                ConfigPriority priority) {
    try {
        fs::path full_path = fs::absolute(file_path);
        
        HAL_INFO("ConfigManager", "Cargando configuración desde archivo",
            {
                {"key", key},
                {"file", full_path.string()},
                {"schema", schema_id}
            });
        
        if (!fs::exists(full_path)) {
            throw ConfigException("Config file not found: " + full_path.string());
        }
        
        // Parsear archivo
        nlohmann::json data = parseFile(full_path);
        
        // Crear metadata
        ConfigMetadata metadata;
        metadata.source = ConfigSource::FILE;
        metadata.priority = priority;
        metadata.created = fs::last_write_time(full_path);
        metadata.modified = metadata.created;
        metadata.checksum = generateChecksum(data);
        
        // Cargar configuración
        loadFromJson(key, data, schema_id, priority, metadata);
        
        // Registrar para auto-reload si está habilitado
        if (auto_reload_) {
            std::lock_guard<std::mutex> lock(callback_mutex_);
            watched_files_[full_path] = fs::last_write_time(full_path);
        }
        
        HAL_INFO("ConfigManager", "Configuración cargada exitosamente",
            {{"key", key}, {"size_bytes", fs::file_size(full_path)}});
        
    } catch (const std::exception& e) {
        HAL_ERROR("ConfigManager",
            std::string("Error cargando configuración: ") + e.what(),
            {{"key", key}, {"file", file_path.string()}});
        throw;
    }
}

void ConfigManager::loadFromJson(const std::string& key,
                                const nlohmann::json& data,
                                const std::string& schema_id,
                                ConfigPriority priority,
                                const ConfigMetadata& metadata) {
    std::unique_lock<std::shared_mutex> lock(store_mutex_);
    
    // Validar si hay schema
    if (!schema_id.empty()) {
        validateAgainstSchema(data, schema_id);
    }
    
    // Verificar si ya existe
    auto it = config_store_.find(key);
    if (it != config_store_.end()) {
        // Verificar prioridad
        if (priority <= it->second.metadata.priority) {
            HAL_WARN("ConfigManager", 
                "Ignorando configuración de menor prioridad",
                {
                    {"key", key},
                    {"existing_priority", static_cast<int>(it->second.metadata.priority)},
                    {"new_priority", static_cast<int>(priority)}
                });
            return;
        }
        
        // Backup del valor anterior
        backupConfig(key, it->second.data);
    }
    
    // Crear nueva entrada
    ConfigEntry entry;
    entry.key = key;
    entry.data = data;
    entry.metadata = metadata;
    entry.loaded_at = std::chrono::steady_clock::now();
    entry.validated = !schema_id.empty();
    entry.schema_id = schema_id;
    
    // Guardar
    config_store_[key] = entry;
    
    HAL_DEBUG("ConfigManager", "Configuración cargada en store",
        {{"key", key}, {"validated", entry.validated}});
}

void ConfigManager::validateAgainstSchema(const nlohmann::json& data,
                                         const std::string& schema_id) {
    std::shared_lock<std::shared_mutex> lock(schema_mutex_);
    
    auto it = schemas_.find(schema_id);
    if (it == schemas_.end()) {
        throw ConfigValidationException(
            "Schema not found: " + schema_id,
            "", schema_id);
    }
    
    // NOTA: En una implementación real, usaríamos una librería
    // de validación JSON Schema como nlohmann::json_schema
    // Por simplicidad, hacemos una validación básica aquí
    
    const nlohmann::json& schema = it->second;
    
    // Verificar tipo si está especificado
    if (schema.contains("type")) {
        std::string expected_type = schema["type"];
        std::string actual_type = data.type_name();
        
        // Mapear nombres de tipos
        std::map<std::string, std::string> type_map = {
            {"string", "string"},
            {"number", "number"},
            {"integer", "number"},
            {"boolean", "boolean"},
            {"object", "object"},
            {"array", "array"},
            {"null", "null"}
        };
        
        if (type_map.count(expected_type) && 
            type_map[expected_type] != actual_type) {
            throw ConfigValidationException(
                "Type mismatch. Expected: " + expected_type + 
                ", Got: " + actual_type,
                "", schema_id);
        }
    }
    
    // Verificar required fields si es objeto
    if (data.is_object() && schema.contains("required")) {
        for (const auto& required_field : schema["required"]) {
            if (!data.contains(required_field.get<std::string>())) {
                throw ConfigValidationException(
                    "Missing required field: " + required_field.get<std::string>(),
                    "", schema_id);
            }
        }
    }
    
    // Validar propiedades si es objeto
    if (data.is_object() && schema.contains("properties")) {
        for (auto& prop : schema["properties"].items()) {
            if (data.contains(prop.key())) {
                // Recursivamente validar si hay sub-schema
                if (prop.value().is_object()) {
                    try {
                        validateAgainstSchema(data[prop.key()], schema_id + "." + prop.key());
                    } catch (const ConfigValidationException& e) {
                        // Re-lanzar con path actualizado
                        throw ConfigValidationException(
                            e.what() + " (at property: " + prop.key() + ")",
                            "", e.getSchemaPath());
                    }
                }
            }
        }
    }
}

nlohmann::json ConfigManager::parseFile(const fs::path& file_path) {
    ConfigFormat format = detectFormat(file_path);
    
    std::ifstream file(file_path);
    if (!file.is_open()) {
        throw ConfigException("Cannot open file: " + file_path.string());
    }
    
    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string content = buffer.str();
    
    try {
        switch (format) {
            case ConfigFormat::JSON:
                return nlohmann::json::parse(content);
                
            case ConfigFormat::XML:
                // NOTA: Requeriría una librería como pugixml o rapidxml
                // Por ahora, lanzar excepción
                throw ConfigException("XML parsing not yet implemented");
                
            case ConfigFormat::YAML:
                // NOTA: Requeriría una librería como yaml-cpp
                throw ConfigException("YAML parsing not yet implemented");
                
            case ConfigFormat::INI:
                throw ConfigException("INI parsing not yet implemented");
                
            default:
                throw ConfigException("Unknown config format");
        }
    } catch (const nlohmann::json::parse_error& e) {
        throw ConfigParseException(
            std::string("JSON parse error: ") + e.what(),
            file_path.string(),
            e.byte);
    }
}

ConfigFormat ConfigManager::detectFormat(const fs::path& file_path) {
    std::string ext = file_path.extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
    
    if (ext == ".json") return ConfigFormat::JSON;
    if (ext == ".xml") return ConfigFormat::XML;
    if (ext == ".yaml" || ext == ".yml") return ConfigFormat::YAML;
    if (ext == ".ini" || ext == ".cfg") return ConfigFormat::INI;
    
    // Por defecto, asumir JSON
    return ConfigFormat::JSON;
}

void ConfigManager::fileWatcherLoop() {
    HAL_INFO("ConfigManager", "File watcher thread started");
    
    while (running_.load()) {
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        std::lock_guard<std::mutex> lock(callback_mutex_);
        
        for (auto& pair : watched_files_) {
            const fs::path& file_path = pair.first;
            auto& last_write_time = pair.second;
            
            try {
                if (!fs::exists(file_path)) {
                    HAL_WARN("ConfigManager", 
                        "Watched file disappeared",
                        {{"file", file_path.string()}});
                    continue;
                }
                
                auto current_write_time = fs::last_write_time(file_path);
                if (current_write_time > last_write_time) {
                    HAL_INFO("ConfigManager", 
                        "Config file modified, reloading",
                        {{"file", file_path.string()}});
                    
                    // Encontrar la key asociada con este archivo
                    // (necesitaríamos mantener un mapa inverso)
                    // Por simplicidad, recargaríamos todas las configs
                    
                    last_write_time = current_write_time;
                }
            } catch (const std::exception& e) {
                HAL_ERROR("ConfigManager",
                    std::string("Error watching file: ") + e.what(),
                    {{"file", file_path.string()}});
            }
        }
    }
    
    HAL_INFO("ConfigManager", "File watcher thread stopped");
}

void ConfigManager::notifyChangeCallbacks(const std::string& key,
                                         const nlohmann::json& old_value,
                                         const nlohmann::json& new_value,
                                         const ConfigMetadata& metadata) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    
    auto it = change_callbacks_.find(key);
    if (it != change_callbacks_.end()) {
        for (const auto& callback : it->second) {
            try {
                callback(key, old_value, new_value, metadata);
            } catch (const std::exception& e) {
                HAL_ERROR("ConfigManager",
                    std::string("Error in config change callback: ") + e.what(),
                    {{"key", key}});
            }
        }
    }
    
    // También notificar callbacks globales (key vacía)
    auto global_it = change_callbacks_.find("");
    if (global_it != change_callbacks_.end()) {
        for (const auto& callback : global_it->second) {
            try {
                callback(key, old_value, new_value, metadata);
            } catch (const std::exception& e) {
                HAL_ERROR("ConfigManager",
                    std::string("Error in global config change callback: ") + e.what(),
                    {{"key", key}});
            }
        }
    }
}

// ============================================================================
// CONFIGURACIONES ESPECÍFICAS DEL HFSM
// ============================================================================

HFSMConfig ConfigManager::getHFSMConfig(const std::string& config_key) const {
    try {
        auto config_json = getJson(config_key);
        
        HFSMConfig config;
        
        // Mapear campos JSON a HFSMConfig
        if (config_json.contains("state_timeout_ms")) {
            config.state_timeout_ms = config_json["state_timeout_ms"];
        }
        if (config_json.contains("global_timeout_ms")) {
            config.global_timeout_ms = config_json["global_timeout_ms"];
        }
        if (config_json.contains("event_processing_timeout_ms")) {
            config.event_processing_timeout_ms = config_json["event_processing_timeout_ms"];
        }
        if (config_json.contains("loop_sleep_ms")) {
            config.loop_sleep_ms = config_json["loop_sleep_ms"];
        }
        if (config_json.contains("max_consecutive_errors")) {
            config.max_consecutive_errors = config_json["max_consecutive_errors"];
        }
        if (config_json.contains("shutdown_timeout_ms")) {
            config.shutdown_timeout_ms = config_json["shutdown_timeout_ms"];
        }
        if (config_json.contains("enable_watchdog")) {
            config.enable_watchdog = config_json["enable_watchdog"];
        }
        if (config_json.contains("log_transitions")) {
            config.log_transitions = config_json["log_transitions"];
        }
        if (config_json.contains("log_events")) {
            config.log_events = config_json["log_events"];
        }
        
        // Sincronizar campos
        config.syncAll();
        
        return config;
        
    } catch (const std::exception& e) {
        HAL_WARN("ConfigManager",
            std::string("Error parsing HFSM config, using defaults: ") + e.what(),
            {{"config_key", config_key}});
        
        return HFSMConfig::Default();
    }
}

// ============================================================================
// MÉTODOS ESTÁTICOS DE UTILIDAD
// ============================================================================

std::string ConfigManager::formatToString(ConfigFormat format) {
    switch (format) {
        case ConfigFormat::JSON: return "JSON";
        case ConfigFormat::XML: return "XML";
        case ConfigFormat::YAML: return "YAML";
        case ConfigFormat::INI: return "INI";
        default: return "UNKNOWN";
    }
}

std::string ConfigManager::generateChecksum(const nlohmann::json& data) {
    std::string json_str = data.dump();
    
    unsigned char hash[SHA256_DIGEST_LENGTH];
    SHA256(reinterpret_cast<const unsigned char*>(json_str.c_str()), 
           json_str.length(), hash);
    
    std::stringstream ss;
    for (int i = 0; i < SHA256_DIGEST_LENGTH; i++) {
        ss << std::hex << std::setw(2) << std::setfill('0') 
           << static_cast<int>(hash[i]);
    }
    
    return ss.str();
}

nlohmann::json ConfigManager::createDefaultHFSMConfig() {
    return {
        {"state_timeout_ms", 5000},
        {"global_timeout_ms", 30000},
        {"event_processing_timeout_ms", 100},
        {"loop_sleep_ms", 10},
        {"max_consecutive_errors", 10},
        {"shutdown_timeout_ms", 3000},
        {"enable_watchdog", true},
        {"log_transitions", true},
        {"log_events", false},
        {"metadata", {
            {"version", "1.0.0"},
            {"description", "Default HFSM configuration for production"},
            {"author", "System"},
            {"tags", {"production", "default", "robotics"}}
        }}
    };
}

nlohmann::json ConfigManager::createDefaultAlertConfig() {
    return {
        {"thresholds", {
            {
                {"id", "battery_level"},
                {"name", "Battery Level"},
                {"category", "BATTERY"},
                {"warning_value", 20.0},
                {"critical_value", 10.0},
                {"emergency_value", 5.0},
                {"unit", "%"},
                {"auto_acknowledge", true},
                {"triggers", {"LOW_BATTERY", "CRITICAL_BATTERY"}}
            },
            {
                {"id", "cpu_temperature"},
                {"name", "CPU Temperature"},
                {"category", "TEMPERATURE"},
                {"warning_value", 75.0},
                {"critical_value", 85.0},
                {"emergency_value", 95.0},
                {"unit", "°C"},
                {"auto_acknowledge", true},
                {"triggers", {"HIGH_TEMPERATURE"}}
            }
        }},
        {"metadata", {
            {"version", "1.0.0"},
            {"description", "Default alert thresholds for robotics system"},
            {"author", "System"},
            {"tags", {"alerts", "thresholds", "monitoring"}}
        }}
    };
}

// ============================================================================
// IMPLEMENTACIONES DE STRUCTS
// ============================================================================

nlohmann::json ConfigMetadata::toJson() const {
    return {
        {"version", version},
        {"description", description},
        {"author", author},
        {"created", std::chrono::duration_cast<std::chrono::milliseconds>(
            created.time_since_epoch()).count()},
        {"modified", std::chrono::duration_cast<std::chrono::milliseconds>(
            modified.time_since_epoch()).count()},
        {"checksum", checksum},
        {"source", static_cast<int>(source)},
        {"priority", static_cast<int>(priority)},
        {"tags", tags}
    };
}

ConfigMetadata ConfigMetadata::fromJson(const nlohmann::json& j) {
    ConfigMetadata metadata;
    
    metadata.version = j.value("version", "1.0.0");
    metadata.description = j.value("description", "");
    metadata.author = j.value("author", "unknown");
    metadata.checksum = j.value("checksum", "");
    
    if (j.contains("created")) {
        auto ms = j["created"].get<int64_t>();
        metadata.created = std::chrono::system_clock::time_point(
            std::chrono::milliseconds(ms));
    }
    
    if (j.contains("modified")) {
        auto ms = j["modified"].get<int64_t>();
        metadata.modified = std::chrono::system_clock::time_point(
            std::chrono::milliseconds(ms));
    }
    
    metadata.source = static_cast<ConfigSource>(j.value("source", 0));
    metadata.priority = static_cast<ConfigPriority>(j.value("priority", 50));
    
    if (j.contains("tags") && j["tags"].is_object()) {
        for (auto& item : j["tags"].items()) {
            metadata.tags[item.key()] = item.value();
        }
    }
    
    return metadata;
}

nlohmann::json ConfigChange::toJson() const {
    return {
        {"path", path},
        {"old_value", old_value},
        {"new_value", new_value},
        {"timestamp", std::chrono::duration_cast<std::chrono::milliseconds>(
            timestamp.time_since_epoch()).count()},
        {"user", user},
        {"reason", reason}
    };
}

} // namespace ns_fsm
