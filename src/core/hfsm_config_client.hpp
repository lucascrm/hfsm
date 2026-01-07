/**
 * @file     hfsm_config_client.hpp
 * @brief    Cliente unificado para gestión de configuración del sistema HFSM
 * @author   Lucas C.
 * @date     27/12/2025
 * @version  1.0
 * @license  MIT
 * 
 * @details  Este archivo define el cliente de configuración que centraliza el acceso
 *           y gestión de configuración para el sistema HFSM. Proporciona:
 *           - Unificación de ConfigManager, HFSMCore y AlertManager
 *           - Gestión de cambios dinámicos de configuración
 *           - Factory methods para diferentes entornos (producción, desarrollo)
 *           - Persistencia y validación de configuración
 *           - Sistema de callbacks para cambios de configuración
 * 
 * @note     El cliente actúa como fachada para simplificar el acceso a múltiples
 *           subsistemas de configuración. Es responsable de mantener la consistencia
 *           entre diferentes componentes cuando la configuración cambia.
 * 
 * @warning  Cambios de configuración en tiempo real pueden afectar el funcionamiento
 *           del sistema. Usar con precaución en producción.
 * 
 * @see      hfsm_config_manager.hpp - Manejador principal de configuración
 * @see      hfsm_core.hpp - Núcleo HFSM que usa la configuración
 * @see      hfsm_alerts.hpp - Sistema de alertas configurable
 */
#ifndef HFSM_CONFIG_CLIENT_HPP
#define HFSM_CONFIG_CLIENT_HPP

#include "hfsm_config_manager.hpp"
#include "hfsm_core.hpp"
#include "hfsm_alerts.hpp"

namespace ns_fsm {

// ============================================================================
// CLIENTE UNIFICADO DE CONFIGURACIÓN
// ============================================================================

class HFSMConfigClient {
private:
    std::shared_ptr<ConfigManager> config_manager_;
    std::shared_ptr<HFSMCore> hfsm_core_;
    std::shared_ptr<AlertManager> alert_manager_;
    
    std::map<std::string, std::string> config_mappings_;
    std::vector<std::function<void()>> reload_callbacks_;
    
public:
    HFSMConfigClient();
    ~HFSMConfigClient();
    
    // ------------------------------------------------------------------------
    // INICIALIZACIÓN
    // ------------------------------------------------------------------------
    void initialize(const fs::path& config_base_path = "config");
    void loadConfiguration(const std::string& environment = "production");
    void applyConfiguration();
    
    // ------------------------------------------------------------------------
    // REGISTRO DE COMPONENTES
    // ------------------------------------------------------------------------
    void registerHFSM(HFSMCore* hfsm);
    void registerAlertManager(AlertManager* alerts);
    void registerCustomConfig(const std::string& key,
                             const std::string& schema_id,
                             const fs::path& config_file);
    
    // ------------------------------------------------------------------------
    // ACTUALIZACIÓN DINÁMICA
    // ------------------------------------------------------------------------
    void subscribeToConfigChanges();
    void onConfigChanged(const std::string& key,
                        const nlohmann::json& old_value,
                        const nlohmann::json& new_value,
                        const ConfigMetadata& metadata);
    
    // ------------------------------------------------------------------------
    // UTILIDADES
    // ------------------------------------------------------------------------
    void reloadAll();
    void saveCurrentState(const fs::path& backup_dir);
    void validateAllConfigs();
    
    // ------------------------------------------------------------------------
    // ACCESO DIRECTO
    // ------------------------------------------------------------------------
    ConfigManager& getConfigManager();
    HFSMConfig getCurrentHFSMConfig() const;
    std::vector<Threshold> getCurrentAlertThresholds() const;
    
    // ------------------------------------------------------------------------
    // FACTORY METHODS
    // ------------------------------------------------------------------------
    static std::shared_ptr<HFSMConfigClient> createProductionClient();
    static std::shared_ptr<HFSMConfigClient> createDevelopmentClient();
    static std::shared_ptr<HFSMConfigClient> createFromEnvironment();
};

} // namespace ns_fsm

#endif // HFSM_CONFIG_CLIENT_HPP
