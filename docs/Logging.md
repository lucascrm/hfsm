# Documentación de la API HFSM Logging

## Índice

1.  Introducción
2.  Arquitectura del Sistema
3.  API Pública
4.  Guías de Uso
5.  Configuración
6.  Macros de Conveniencia
7.  Manejo de Errores
8.  Rendimiento y Buenas Prácticas

## Introducción

El sistema de logging de HFSM (Hierarchical Finite State Machine) es un
subsistema robusto y de alto rendimiento diseñado para proporcionar
trazabilidad completa del comportamiento de las máquinas de estado.
Ofrece logging estructurado, multi-destino (archivo/consola) y
características avanzadas como rotación de logs y estadísticas en tiempo
de ejecución.

### Características principales:

-   **Logging estructurado** con contexto JSON
-   **Multi-destino**: archivo, consola o ambos
-   **Rotación automática** de archivos de log
-   **Filtrado por nivel** y subsistema
-   **Estadísticas** en tiempo real
-   **Thread-safe** con buffer asíncrono
-   **Configuración dinámica** en tiempo de ejecución

## 

## 

## Arquitectura del Sistema

### Diagrama de componentes:

text

┌─────────────────────────────────────────┐

│ API Pública (Clientes) │

├─────────────────────────────────────────┤

│ Macros de Conveniencia │

│ Funciones Helper │

├─────────────────────────────────────────┤

│ HFSMLogger (Singleton) │

│ ┌─────────────────────────────────┐ │

│ │ Buffer Asíncrono (thread-safe) │ │

│ └─────────────────────────────────┘ │

├─────────────────────────────────────────┤

│ Writers (Salidas) │

│ ┌─────────┐ ┌─────────┐ ┌─────────┐ │

│ │ Archivo │ │ Consola │ │ Red\* │ │

│ └─────────┘ └─────────┘ └─────────┘ │

└─────────────────────────────────────────┘

### Flujo de datos:

1.  Cliente llama a API de logging
2.  Mensaje se encola en buffer thread-safe
3.  Worker thread procesa mensajes asíncronamente
4.  Mensaje se formatea y envía a los destinos configurados

## API Pública

### Clase Principal: *HFSMLogger*

#### Inicialización y Control

cpp

// Obtener instancia singleton

static HFSMLogger& getInstance();

// Inicializar logger con configuración

void initialize(const LogConfig& config = LogConfig());

// Apagar logger (limpia buffers, cierra archivos)

void shutdown();

#### Métodos de Logging

**Logging estructurado genérico:**

cpp

void logStructured(const std::string& subsystem,

hardware::DiagnosticLevel level,

const std::string& message,

const nlohmann::json& context = {});

**Métodos específicos para HFSM:**

cpp

// Transiciones de estado

void logStateTransition(uint32_t from_state, uint32_t to_state,

const std::string& trigger = \"\",

bool success = true);

// Eventos procesados

void logEventProcessed(const Event& event, bool success = true,

const std::string& handler = \"\");

// Errores del sistema

void logError(ErrorCode code, const std::string& context,

const std::string& component = \"HFSM\");

// Métricas de rendimiento

void logPerformanceMetric(const std::string& metric_name,

double value,

const std::string& unit = \"\");

#### Configuración

cpp

// Cambiar nivel mínimo de log dinámicamente

void setMinLogLevel(hardware::DiagnosticLevel level);

#### Estadísticas

cpp

// Obtener estadísticas en formato JSON

nlohmann::json getStatistics() const;

### Estructura *LogConfig*

cpp

struct LogConfig {

bool enable_structured_logging; // Habilitar logging estructurado

bool enable_file_logging; // Escribir a archivo

bool enable_console_logging; // Escribir a consola

size_t max_log_size_mb; // Tamaño máximo por archivo (MB)

size_t max_log_files; // Número máximo de archivos rotados

hardware::DiagnosticLevel min_level; // Nivel mínimo de log

std::vector\<std::string\> filtered_subsystems; // Subsistemas a filtrar

// Constructor con valores por defecto

LogConfig();

};

### Niveles de Log (*hardware::DiagnosticLevel*)

cpp

enum class DiagnosticLevel {

DEBUG, // Información detallada para debugging

INFO, // Información general del sistema

WARN, // Advertencias no críticas

ERROR, // Errores recuperables

FATAL // Errores críticos que requieren intervención

};

## Guías de Uso

### 1. Configuración Básica

cpp

#include \"hfsm_logging.hpp\"

int main() {

// Obtener instancia del logger

auto& logger = ns_fsm::HFSMLogger::getInstance();

// Configurar logger

ns_fsm::LogConfig config;

config.enable_structured_logging = true;

config.enable_file_logging = true;

config.enable_console_logging = true;

config.max_log_size_mb = 10;

config.max_log_files = 5;

config.min_level = hardware::DiagnosticLevel::INFO;

// Inicializar

logger.initialize(config);

// Usar logger\...

// Apagar al finalizar

logger.shutdown();

return 0;

}

### 2. Logging Estructurado

cpp

// Log básico con contexto

logger.logStructured(

\"Navigation\",

hardware::DiagnosticLevel::INFO,

\"Iniciando navegación autónoma\",

{

{\"target\", \"Waypoint_A\"},

{\"speed\", 2.5},

{\"sensors\", {\"lidar\", \"camera\", \"imu\"}}

}

);

// Log de error con contexto

logger.logStructured(

\"MotorController\",

hardware::DiagnosticLevel::ERROR,

\"Fallo en motor izquierdo\",

{

{\"motor_id\", \"MTR_LEFT_01\"},

{\"error_code\", 0x45},

{\"temperature\", 85.5},

{\"rpm\", 0}

}

);

### 3. Logging Específico de HFSM

cpp

// Registrar transición de estado

logger.logStateTransition(

static_cast\<uint32_t\>(StateType::READY),

static_cast\<uint32_t\>(StateType::OPERATIONAL),

\"USER_START_COMMAND\",

true

);

// Registrar evento procesado

Event nav_event(EventType::ROBOT_NAVIGATION_START, \"Go to kitchen\");

logger.logEventProcessed(nav_event, true, \"NavigationHandler\");

// Registrar error

logger.logError(

ErrorCode::SENSOR_TIMEOUT,

\"Timeout en comunicación con sensor LIDAR\",

\"SensorManager\"

);

// Registrar métrica de rendimiento

logger.logPerformanceMetric(

\"loop_execution_time\",

45.3,

\"ms\"

);

### 4. Usando las Funciones Helper

cpp

#include \"hfsm_logging_helpers.hpp\"

// Más conciso y legible

ns_fsm::log_debug_helper(\"SensorFusion\", \"Fusión de datos
completada\");

ns_fsm::log_info_helper(\"Navigation\", \"Destino alcanzado\");

ns_fsm::log_warn_helper(\"Battery\", \"Batería baja: 15%\");

ns_fsm::log_error_helper(\"Motor\", \"Sobrecorriente detectada\");

ns_fsm::log_fatal_helper(\"System\", \"Pérdida de comunicación
crítica\");

### 5. Monitoreo y Estadísticas

cpp

// Obtener estadísticas en cualquier momento

auto stats = logger.getStatistics();

// Ejemplo de estadísticas disponibles

std::cout \<\< \"Total logs: \" \<\< stats\[\"total_logs\"\] \<\<
std::endl;

std::cout \<\< \"Logs por nivel:\" \<\< std::endl;

std::cout \<\< \" DEBUG: \" \<\< stats\[\"levels\"\]\[\"DEBUG\"\] \<\<
std::endl;

std::cout \<\< \" INFO: \" \<\< stats\[\"levels\"\]\[\"INFO\"\] \<\<
std::endl;

std::cout \<\< \" WARN: \" \<\< stats\[\"levels\"\]\[\"WARN\"\] \<\<
std::endl;

std::cout \<\< \" ERROR: \" \<\< stats\[\"levels\"\]\[\"ERROR\"\] \<\<
std::endl;

std::cout \<\< \" FATAL: \" \<\< stats\[\"levels\"\]\[\"FATAL\"\] \<\<
std::endl;

std::cout \<\< \"Buffer size: \" \<\< stats\[\"buffer_size\"\] \<\<
std::endl;

## Configuración

### Configuración por Defecto

cpp

LogConfig config_default; // Valores por defecto:

// enable_structured_logging = true

// enable_file_logging = true

// enable_console_logging = true

// max_log_size_mb = 10

// max_log_files = 5

// min_level = hardware::DiagnosticLevel::INFO

// filtered_subsystems = {} (vacío)

### Configuraciones Comunes

**Para desarrollo:**

cpp

ns_fsm::LogConfig dev_config;

dev_config.enable_structured_logging = true;

dev_config.enable_file_logging = false;

dev_config.enable_console_logging = true;

dev_config.min_level = hardware::DiagnosticLevel::DEBUG;

**Para producción:**

cpp

ns_fsm::LogConfig prod_config;

prod_config.enable_structured_logging = true;

prod_config.enable_file_logging = true;

prod_config.enable_console_logging = false;

prod_config.max_log_size_mb = 50;

prod_config.max_log_files = 10;

prod_config.min_level = hardware::DiagnosticLevel::WARN;

**Para pruebas unitarias:**

cpp

ns_fsm::LogConfig test_config;

test_config.enable_structured_logging = false;

test_config.enable_file_logging = false;

test_config.enable_console_logging = true;

test_config.min_level = hardware::DiagnosticLevel::ERROR;

### Configuración Dinámica

cpp

// Cambiar nivel de log en tiempo de ejecución

logger.setMinLogLevel(hardware::DiagnosticLevel::DEBUG);

// Reinicializar con nueva configuración

logger.shutdown();

logger.initialize(new_config);

## Macros de Conveniencia

### Macros para HFSM

cpp

// Transición de estado

HFSM_LOG_TRANSITION(from_state, to_state, trigger, success);

// Evento procesado

HFSM_LOG_EVENT(event, success, handler);

// Error

HFSM_LOG_ERROR(code, context, component);

// Métrica de rendimiento

HFSM_LOG_PERFORMANCE(name, value, unit);

**Ejemplo de uso:**

cpp

// En un handler de estado

void onEnterOperational() {

HFSM_LOG_TRANSITION(

StateType::READY,

StateType::OPERATIONAL,

\"START_BUTTON_PRESSED\",

true

);

HFSM_LOG_PERFORMANCE(\"init_time\", 125.4, \"ms\");

}

// En un handler de eventos

void handleNavigationEvent(const Event& event) {

HFSM_LOG_EVENT(event, true, \"NavigationHandler\");

if (event.type == EventType::COLLISION_DETECTED) {

HFSM_LOG_ERROR(

ErrorCode::COLLISION_DETECTED,

\"Obstáculo detectado frontal\",

\"CollisionAvoidance\"

);

}

}

### Macros para Medición de Scope

cpp

// Medir tiempo de ejecución de un scope

HFSM_LOG_SCOPE_START(DataProcessing);

// \... código a medir \...

HFSM_LOG_SCOPE_END(DataProcessing);

**Salida generada:**

text

\[DEBUG\] \[HFSM\] Entering scope: DataProcessing

\[PERF\] scope_DataProcessing_duration: 45.2 ms

\[DEBUG\] \[HFSM\] Exiting scope: DataProcessing (took 45.2ms)

## Manejo de Errores

### Códigos de Error Comunes

  ------------------------ ---------------------------------- ------------------------------------------------
  *LOG_FILE_OPEN_FAILED*   No se puede abrir archivo de log   Verificar permisos, espacio en disco
  *LOG_BUFFER_OVERFLOW*    Buffer de logs lleno               Aumentar tamaño de buffer o reducir verbosidad
  *LOG_WRITE_FAILED*       Error al escribir log              Verificar sistema de archivos
  *LOG_ROTATION_FAILED*    Error al rotar logs                Limpiar manualmente archivos viejos
  ------------------------ ---------------------------------- ------------------------------------------------

### Ejemplo de Manejo de Errores

cpp

try {

auto& logger = ns_fsm::HFSMLogger::getInstance();

logger.initialize(config);

} catch (const std::exception& e) {

// Fallback a logging básico

std::cerr \<\< \"ERROR inicializando logger: \" \<\< e.what() \<\<
std::endl;

// Continuar sin logging estructurado

}

### Recuperación Automática

El logger incluye mecanismos de recuperación automática:

1.  **Reintento** en fallos de escritura
2.  **Fallback** a consola si falla escritura a archivo
3.  **Auto-rotación** cuando se alcanza tamaño máximo
4.  **Limpiado** de archivos viejos

## Rendimiento y Buenas Prácticas

### Optimización de Rendimiento

1.  **Usar nivel apropiado:** No usar DEBUG en producción

    cpp

// MAL: Demasiado verboso

logger.logStructured(\"Sensor\", DEBUG, \"Raw value: \" +
std::to_string(value));

// BIEN: Solo cuando sea necesario

if (value \> threshold) {

logger.logStructured(\"Sensor\", WARN, \"Valor alto detectado\",
{{\"value\", value}});

}

**Evitar formato en llamadas críticas:**

cpp

// MAL: Formato en línea caliente

logger.logInfo(\"Valor: \" + std::to_string(complexCalculation()));

// BIEN: Pre-calcular

double result = complexCalculation();

logger.logInfo(\"Valor: \" + std::to_string(result));

**Usar contexto JSON eficientemente:**

cpp

// BIEN: Mover construcción fuera del loop

nlohmann::json context = {

{\"sensor_id\", sensor_id},

{\"sampling_rate\", 100}

};

for (auto& reading : readings) {

context\[\"value\"\] = reading;

logger.logStructured(\"Sensor\", INFO, \"Reading\", context);

}

### Buenas Prácticas

1.  **Subsistemas significativos:**

    cpp

// MAL: Subsistemas genéricos

logger.logStructured(\"System\", INFO, \"Mensaje\");

// BIEN: Subsistemas específicos

logger.logStructured(\"Navigation.PathPlanner\", INFO, \"Ruta
calculada\");

**Mensajes informativos:**

cpp

// MAL: Poco informativo

logger.logStructured(\"Motor\", ERROR, \"Error\");

// BIEN: Descriptivo

logger.logStructured(\"Motor.LeftWheel\", ERROR, \"Encoder fault
detected\",

{{\"error_code\", 0x45}, {\"rpm\", 0}, {\"temperature\", 85.5}});

**Consistencia en niveles:**

cpp

// Guía de niveles

DEBUG: Desarrollo, diagnóstico detallado

INFO: Eventos normales del sistema

WARN: Situaciones anómalas pero manejables

ERROR: Errores que requieren atención

FATAL: Errores críticos que detienen sistema

### Configuración para Diferentes Escenarios

**Alto Rendimiento (Robótica en tiempo real):**

cpp

ns_fsm::LogConfig rt_config;

rt_config.enable_structured_logging = true;

rt_config.enable_file_logging = true;

rt_config.enable_console_logging = false;

rt_config.max_log_size_mb = 100; // Logs más grandes, menos rotación

rt_config.min_level = hardware::DiagnosticLevel::WARN; // Solo problemas

**Debugging Intensivo:**

cpp

ns_fsm::LogConfig debug_config;

debug_config.enable_structured_logging = true;

debug_config.enable_file_logging = true;

debug_config.enable_console_logging = true;

debug_config.max_log_size_mb = 10; // Rotación frecuente

debug_config.max_log_files = 20; // Mantener más archivos

debug_config.min_level = hardware::DiagnosticLevel::DEBUG;

## Ejemplos Complejos

### Ejemplo 1: Sistema Robótico Completo

cpp

class RobotController {

private:

ns_fsm::HFSMLogger& logger\_;

public:

RobotController() : logger\_(ns_fsm::HFSMLogger::getInstance()) {

// Configuración específica para robot

ns_fsm::LogConfig config;

config.enable_structured_logging = true;

config.enable_file_logging = true;

config.enable_console_logging = false;

config.max_log_size_mb = 50;

config.min_level = hardware::DiagnosticLevel::INFO;

logger\_.initialize(config);

}

void performMission() {

HFSM_LOG_SCOPE_START(RobotMission);

// Inicio de misión

logger\_.logStructured(

\"MissionControl\",

hardware::DiagnosticLevel::INFO,

\"Iniciando misión autónoma\",

{

{\"mission_id\", \"M2024_001\"},

{\"waypoints\", 5},

{\"estimated_time\", \"30m\"}

}

);

// Navegación

HFSM_LOG_TRANSITION(

StateType::STATIONARY,

StateType::NAVIGATING,

\"MISSION_START\",

true

);

// Monitoreo continuo

monitorSensors();

HFSM_LOG_SCOPE_END(RobotMission);

}

private:

void monitorSensors() {

while (mission_active) {

auto sensor_data = readSensors();

if (sensor_data.battery_level \< 20.0) {

logger\_.logStructured(

\"PowerSystem\",

hardware::DiagnosticLevel::WARN,

\"Batería baja\",

{{\"level\", sensor_data.battery_level}}

);

}

if (sensor_data.collision_detected) {

HFSM_LOG_ERROR(

ErrorCode::COLLISION_DETECTED,

\"Obstáculo detectado\",

\"CollisionSensor\"

);

}

// Métrica de rendimiento

HFSM_LOG_PERFORMANCE(

\"sensor_loop_time\",

sensor_data.processing_time,

\"ms\"

);

}

}

};

### Ejemplo 2: Sistema de Monitorización de Salud

cpp

class HealthMonitor {

public:

void checkSystemHealth() {

auto& logger = ns_fsm::HFSMLogger::getInstance();

auto health = collectHealthMetrics();

nlohmann::json health_context = {

{\"timestamp\", getCurrentTime()},

{\"metrics\", {

{\"cpu_usage\", health.cpu_usage},

{\"memory_usage\", health.memory_usage},

{\"disk_usage\", health.disk_usage},

{\"temperature\", health.temperature}

}}

};

// Determinar nivel basado en métricas

auto level = hardware::DiagnosticLevel::INFO;

std::string message = \"Sistema saludable\";

if (health.cpu_usage \> 90.0 \|\| health.temperature \> 80.0) {

level = hardware::DiagnosticLevel::ERROR;

message = \"Sistema bajo estrés\";

} else if (health.cpu_usage \> 70.0) {

level = hardware::DiagnosticLevel::WARN;

message = \"Uso de CPU elevado\";

}

logger.logStructured(

\"HealthMonitor\",

level,

message,

health_context

);

// Log detallado si hay problemas

if (level \>= hardware::DiagnosticLevel::WARN) {

logger.logStructured(

\"HealthMonitor.Detailed\",

hardware::DiagnosticLevel::DEBUG,

\"Métricas detalladas de salud\",

health_context

);

}

}

};

## 

## 

## Conclusión

El sistema de logging HFSM proporciona una solución robusta, flexible y
de alto rendimiento para el registro de eventos en sistemas basados en
máquinas de estado. Su diseño modular y API bien definida permiten:

1.  **Integración sencilla** en sistemas existentes
2.  **Configuración flexible** para diferentes entornos
3.  **Alto rendimiento** con logging asíncrono
4.  **Trazabilidad completa** con logging estructurado
5.  **Mantenimiento simplificado** con rotación automática

Siguiendo las guías y mejores prácticas presentadas, los desarrolladores
pueden implementar un sistema de logging efectivo que proporcione
visibilidad valiosa mientras mantiene un impacto mínimo en el
rendimiento del sistema.
