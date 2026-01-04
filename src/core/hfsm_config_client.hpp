// hfsm_config_client.hpp
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
