#include "ProjectManager.hpp"
#include "Registry.hpp"
#include "Component.hpp"
#include "Port.hpp"
#include "CatalogProvider.hpp"
#include "physics/PhysicsWorld.hpp"
#include "core/SystemConfig.hpp"
#include <imgui.h>
#include <spdlog/spdlog.h>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <random>
#include <set>

namespace mechatron {

namespace fs = std::filesystem;

static constexpr const char* PROJECT_CIRCUIT_FILE = "circuit/circuit.json";
static constexpr const char* PROJECT_MODELS_FILE = "models/models.json";
static constexpr const char* PROJECT_CODE_STATE_FILE = "code/code_editor.json";
static constexpr const char* PROJECT_SKETCH_FILE = "code/sketch.ino";

// Hash function for change detection (FNV-1a 64-bit)
static uint64_t fnv1a64_hash(const void* data, size_t size) {
    uint64_t hash = 14695981039346656037ULL;
    const uint8_t* bytes = static_cast<const uint8_t*>(data);
    for (size_t i = 0; i < size; ++i) {
        hash ^= bytes[i];
        hash *= 1099511628211ULL;
    }
    return hash;
}

static std::string resolve_plugin_for_component_type(SimulationOrchestrator& orchestrator,
                                                     const std::string& preferred_plugin,
                                                     const std::string& component_type) {
    if (!preferred_plugin.empty()) {
        if (auto* plugin = orchestrator.plugin_host().get_plugin(preferred_plugin)) {
            for (const auto& desc : plugin->components()) {
                if (desc.type == component_type) {
                    return preferred_plugin;
                }
            }
        }
    }

    for (auto* plugin : orchestrator.plugin_host().get_all_plugins()) {
        if (!plugin) continue;
        for (const auto& desc : plugin->components()) {
            if (desc.type == component_type) {
                return std::string(plugin->name());
            }
        }
    }

    return preferred_plugin;
}

static bool write_json_file(const fs::path& path, const nlohmann::json& value) {
    std::error_code ec;
    fs::create_directories(path.parent_path(), ec);
    if (ec) return false;

    std::ofstream file(path);
    if (!file.is_open()) return false;

    try {
        file << std::setw(2) << value;
    } catch (...) {
        return false;
    }
    return true;
}

static bool read_json_file(const fs::path& path, nlohmann::json& value) {
    std::ifstream file(path);
    if (!file.is_open()) return false;
    try {
        file >> value;
    } catch (...) {
        return false;
    }
    return true;
}

static bool write_text_file(const fs::path& path, const std::string& text) {
    std::error_code ec;
    fs::create_directories(path.parent_path(), ec);
    if (ec) return false;

    std::ofstream file(path);
    if (!file.is_open()) return false;
    file << text;
    return static_cast<bool>(file);
}

static bool read_text_file(const fs::path& path, std::string& text) {
    std::ifstream file(path);
    if (!file.is_open()) return false;
    std::ostringstream buffer;
    buffer << file.rdbuf();
    text = buffer.str();
    return true;
}

static fs::path normalized_absolute_path(const fs::path& path) {
    std::error_code ec;
    fs::path absolute_path = path.is_absolute() ? path : fs::absolute(path, ec);
    if (ec) {
        absolute_path = path;
    }

    fs::path normalized = fs::weakly_canonical(absolute_path, ec);
    if (ec) {
        normalized = absolute_path.lexically_normal();
    }
    return normalized;
}

static bool project_relative_path(const fs::path& project_dir,
                                  const fs::path& candidate,
                                  fs::path& relative_out) {
    std::error_code ec;
    const fs::path base = normalized_absolute_path(project_dir);
    const fs::path child = normalized_absolute_path(candidate);
    fs::path relative = fs::relative(child, base, ec);
    if (ec || relative.empty()) {
        return false;
    }
    for (const auto& part : relative) {
        if (part == "..") {
            return false;
        }
    }
    relative_out = relative;
    return true;
}

static bool same_normalized_path(const fs::path& a, const fs::path& b) {
    return normalized_absolute_path(a) == normalized_absolute_path(b);
}

static Port* find_component_port(Component* component, const std::string& name) {
    if (!component) return nullptr;
    const std::string role_pin = SystemConfig::instance().get_pin_name_by_role(component->component_type(), name);
    if (!role_pin.empty()) {
        for (Port* port : component->get_ports()) {
            if (port && port->name() == role_pin) return port;
        }
    }
    for (Port* port : component->get_ports()) {
        if (port && port->name() == name) {
            return port;
        }
    }
    return nullptr;
}

static std::vector<nlohmann::json> catalog_project_templates() {
    std::vector<nlohmann::json> templates;
    const auto catalog = default_catalog_provider().load_catalog();
    if (!catalog || !catalog->contains("project_templates") || !(*catalog)["project_templates"].is_array()) {
        return templates;
    }

    for (const auto& entry : (*catalog)["project_templates"]) {
        if (entry.is_object() && entry.contains("id") && entry.contains("display_name")) {
            templates.push_back(entry);
        }
    }

    std::stable_sort(templates.begin(), templates.end(), [](const auto& a, const auto& b) {
        return a.value("order", 0) < b.value("order", 0);
    });
    return templates;
}

static bool template_matches(const nlohmann::json& templ, const std::string& requested_name) {
    if (requested_name.empty()) {
        return templ.value("id", "") == "empty";
    }
    if (templ.value("id", "") == requested_name || templ.value("display_name", "") == requested_name) {
        return true;
    }
    const auto aliases = templ.find("aliases");
    if (aliases != templ.end() && aliases->is_array()) {
        for (const auto& alias : *aliases) {
            if (alias.is_string() && alias.get<std::string>() == requested_name) {
                return true;
            }
        }
    }
    return false;
}

static std::optional<nlohmann::json> find_catalog_template(const std::string& requested_name) {
    for (const auto& templ : catalog_project_templates()) {
        if (template_matches(templ, requested_name)) {
            return templ;
        }
    }
    return std::nullopt;
}

static Vec3 vec3_from_json(const nlohmann::json& value, Vec3 fallback = Vec3{}) {
    if (!value.is_array() || value.size() < 3) {
        return fallback;
    }
    return Vec3{
        value.at(0).get<float>(),
        value.at(1).get<float>(),
        value.at(2).get<float>()
    };
}

static std::string find_type_by_capability(const std::string& capability) {
    for (const auto& type : SystemConfig::instance().all_type_ids()) {
        if (SystemConfig::instance().has_capability(type, capability)) {
            return std::string(type);
        }
    }
    return {};
}

static std::string resolve_template_component_type(const nlohmann::json& spec) {
    if (spec.contains("type") && spec["type"].is_string()) {
        return spec["type"].get<std::string>();
    }
    if (spec.contains("capability") && spec["capability"].is_string()) {
        return find_type_by_capability(spec["capability"].get<std::string>());
    }
    return {};
}

ProjectManager::ProjectManager(SimulationOrchestrator& orchestrator)
    : m_orchestrator(orchestrator)
    , m_last_autosave_time(0.0)
    , m_last_autosave_hash(0)
{
    // Initialize default session state
    m_session.camera.position = Vec3{10.0f, 10.0f, 10.0f};
    m_session.camera.target = Vec3{0.0f, 0.0f, 0.0f};
    m_session.camera.up = Vec3{0.0f, 1.0f, 0.0f};
}

ProjectManager::~ProjectManager() {
    if (m_project_open && m_modified) {
        // Attempt autosave on exit if modified
        if (m_autosave_config.enabled) {
            perform_autosave();
        }
    }
}

bool ProjectManager::create_new_project(const std::string& name, const std::string& location) {
    if (m_project_open) {
        close_project();
    }
    m_orchestrator.clear_scene();

    // Validate and create project directory
    fs::path project_dir = fs::path(location) / sanitize_filename(name);
    try {
        fs::create_directories(project_dir);
    } catch (const std::exception& e) {
        return false;
    }

    // Initialize project
    m_metadata = ProjectMetadata();
    m_metadata.name = name;
    m_metadata.modified = m_metadata.created;

    // Create project file path
    m_project_path = (project_dir / (sanitize_filename(name) + PROJECT_EXTENSION)).string();

    // Initialize session state
    m_session = SessionState();
    m_session.camera.position = Vec3{10.0f, 10.0f, 10.0f};
    m_session.camera.target = Vec3{0.0f, 0.0f, 0.0f};
    m_session.camera.up = Vec3{0.0f, 1.0f, 0.0f};

    m_project_open = true;
    m_modified = false;
    update_statistics();

    if (m_on_deserialize_application_state) {
        m_on_deserialize_application_state(nlohmann::json{
            {"circuit_editor", nlohmann::json::object()},
            {"model_editor", nlohmann::json::object()},
            {"code_editor", nlohmann::json{
                {"content", ""},
                {"current_file", ""},
                {"file_modified", false},
                {"selected_mcu_id", ""},
                {"compiled_hex_path", ""}
            }}
        });
    }

    // Save empty project
    if (!save_project()) {
        close_project();
        return false;
    }

    if (m_on_project_changed) {
        m_on_project_changed(m_project_path);
    }

    return true;
}

bool ProjectManager::open_project(const std::string& project_path) {
    if (m_project_open) {
        close_project();
    }

    // Check if file exists
    if (!fs::exists(project_path)) {
        return false;
    }

    // Load and parse JSON
    std::ifstream file(project_path);
    if (!file.is_open()) {
        return false;
    }

    nlohmann::json j;
    try {
        file >> j;
    } catch (const std::exception& e) {
        return false;
    }

    // Validate format
    if (j.value("format", "") != PROJECT_FORMAT) {
        return false;
    }

    m_orchestrator.clear_scene();

    m_project_path = project_path;

    // Deserialize project
    if (!deserialize_from_json(j)) {
        m_project_path.clear();
        return false;
    }

    m_project_open = true;
    m_modified = false;

    // Restore session state
    restore_session_state();

    update_statistics();

    if (m_on_project_changed) {
        m_on_project_changed(m_project_path);
    }

    return true;
}

bool ProjectManager::save_project() {
    if (!m_project_open || m_project_path.empty()) {
        return false;
    }

    // Update modification time
    auto now = std::time(nullptr);
    char buf[100];
    std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", std::gmtime(&now));
    m_metadata.modified = buf;

    // Serialize to JSON
    nlohmann::json j;
    if (!serialize_to_json(j, true)) {
        return false;
    }

    // Save to file
    std::ofstream file(m_project_path);
    if (!file.is_open()) {
        return false;
    }

    try {
        file << std::setw(2) << j;
    } catch (const std::exception& e) {
        return false;
    }

    m_modified = false;

    if (m_on_modification_changed) {
        m_on_modification_changed(false);
    }

    // Clear autosaves after successful save
    cleanup_old_autosaves();

    return true;
}

bool ProjectManager::save_project_as(const std::string& project_path) {
    if (!m_project_open) {
        return false;
    }

    std::string old_path = m_project_path;
    std::string old_name = m_metadata.name;
    m_project_path = project_path;
    fs::path p(project_path);
    std::error_code ec;
    if (!p.parent_path().empty()) {
        fs::create_directories(p.parent_path(), ec);
        if (ec) {
            m_project_path = old_path;
            m_metadata.name = old_name;
            return false;
        }
    }
    m_metadata.name = p.stem().string();

    if (save_project()) {
        return true;
    } else {
        m_project_path = old_path;
        m_metadata.name = old_name;
        return false;
    }
}

void ProjectManager::close_project() {
    if (!m_project_open) {
        return;
    }

    // Prompt for save if modified (handled by UI)

    m_orchestrator.clear_scene();
    m_project_open = false;
    m_modified = false;
    m_project_path.clear();
    m_metadata = ProjectMetadata();
    m_session = SessionState();
    m_stats = ProjectStats();

    if (m_on_project_changed) {
        m_on_project_changed("");
    }
}

bool ProjectManager::has_autosave() const {
    if (!m_project_open) {
        return false;
    }

    auto autosaves = get_existing_autosaves();
    return !autosaves.empty();
}

void ProjectManager::update_autosave(double current_time_seconds) {
    if (!m_project_open || !m_autosave_config.enabled) return;
    if (current_time_seconds - m_last_autosave_time < m_autosave_config.interval_seconds) return;

    const uint64_t current_hash = calculate_project_hash();
    if (!m_modified && current_hash == m_last_autosave_hash) return;

    perform_autosave();
}

bool ProjectManager::restore_from_autosave() {
    return restore_from_autosave(0);
}

bool ProjectManager::restore_from_autosave(size_t index) {
    if (!has_autosave()) {
        return false;
    }

    auto autosaves = get_existing_autosaves();
    if (autosaves.empty() || index >= autosaves.size()) {
        return false;
    }

    fs::path autosave_path = autosaves[index];

    std::ifstream file(autosave_path);
    if (!file.is_open()) {
        return false;
    }

    nlohmann::json j;
    try {
        file >> j;
    } catch (const std::exception& e) {
        return false;
    }

    // Deserialize project from autosave
    if (!deserialize_from_json(j)) {
        return false;
    }

    m_modified = true;  // Mark as modified after restore
    update_statistics();

    if (m_on_modification_changed) {
        m_on_modification_changed(true);
    }

    return true;
}

std::vector<std::string> ProjectManager::list_autosaves() const {
    std::vector<std::string> autosaves;
    for (const auto& path : get_existing_autosaves()) {
        autosaves.push_back(path.string());
    }
    return autosaves;
}

void ProjectManager::clear_autosaves() {
    auto autosave_dir = get_autosave_directory();
    if (fs::exists(autosave_dir)) {
        fs::remove_all(autosave_dir);
    }
}

std::vector<std::string> ProjectManager::get_available_templates() {
    std::vector<std::string> templates;
    for (const auto& templ : catalog_project_templates()) {
        templates.push_back(templ.value("display_name", templ.value("id", "")));
    }
    return templates;
}

bool ProjectManager::create_from_template(const std::string& template_name,
                                         const std::string& project_name,
                                         const std::string& location) {
    const auto templ = find_catalog_template(template_name);
    if (!templ) {
        return false;
    }

    // Create new project
    if (!create_new_project(project_name, location)) {
        return false;
    }

    auto fail_template = [this]() {
        close_project();
        return false;
    };

    const auto recipe = templ->value("recipe", nlohmann::json::object());
    const auto components = recipe.value("components", nlohmann::json::array());
    const auto connections = recipe.value("connections", nlohmann::json::array());

    if (!components.is_array() || !connections.is_array()) {
        return fail_template();
    }

    for (const auto& spec : components) {
        if (!spec.is_object() || !spec.contains("id") || !spec["id"].is_string()) {
            return fail_template();
        }

        const std::string id = spec["id"].get<std::string>();
        const std::string type = resolve_template_component_type(spec);
        if (id.empty() || type.empty()) {
            return fail_template();
        }

        std::string plugin = spec.value("plugin", "");
        plugin = resolve_plugin_for_component_type(m_orchestrator, plugin, type);
        if (plugin.empty()) {
            return fail_template();
        }

        auto* component = m_orchestrator.create_component(plugin, type, id);
        if (!component) {
            return fail_template();
        }
        if (spec.contains("position")) {
            component->transform().position = vec3_from_json(spec["position"], component->transform().position);
        }
    }

    for (const auto& spec : connections) {
        if (!spec.is_object() ||
            !spec.contains("from") || !spec["from"].is_string() ||
            !spec.contains("to") || !spec["to"].is_string()) {
            return fail_template();
        }

        auto* from_component = m_orchestrator.registry().get(spec["from"].get<std::string>());
        auto* to_component = m_orchestrator.registry().get(spec["to"].get<std::string>());

        const std::string from_pin = spec.value("from_pin", spec.value("from_role", ""));
        const std::string to_pin = spec.value("to_pin", spec.value("to_role", ""));
        if (!from_component || !to_component || from_pin.empty() || to_pin.empty()) {
            return fail_template();
        }

        if (!m_orchestrator.connect(find_component_port(from_component, from_pin),
                                    find_component_port(to_component, to_pin),
                                    spec.value("id", ""))) {
            return fail_template();
        }
    }

    if (!save_project()) {
        return fail_template();
    }
    return true;
}

bool ProjectManager::export_project(const std::string& export_path) {
    if (!m_project_open) {
        return false;
    }

    if (!save_project()) {
        return false;
    }

    // Create export package
    fs::path export_dir(export_path);

    try {
        fs::create_directories(export_dir);

        // Copy project file
        fs::path project_file = export_dir / (m_metadata.name + PROJECT_EXTENSION);
        fs::copy_file(m_project_path, project_file, fs::copy_options::overwrite_existing);

        const fs::path project_dir = fs::path(m_project_path).parent_path();
        for (const auto* dir_name : {"circuit", "models", "code"}) {
            const fs::path src = project_dir / dir_name;
            const fs::path dst = export_dir / dir_name;
            if (fs::exists(src)) {
                fs::copy(src, dst,
                         fs::copy_options::recursive |
                         fs::copy_options::overwrite_existing);
            }
        }

        return true;
    } catch (const std::exception& e) {
        return false;
    }
}

bool ProjectManager::import_project(const std::string& import_path) {
    fs::path source(import_path);
    if (fs::is_regular_file(source)) {
        return open_project(source.string());
    }

    if (!fs::is_directory(source)) {
        return false;
    }

    std::vector<fs::path> candidates;
    try {
        for (const auto& entry : fs::directory_iterator(source)) {
            if (entry.is_regular_file() && entry.path().extension() == PROJECT_EXTENSION) {
                candidates.push_back(entry.path());
            }
        }
    } catch (...) {
        return false;
    }

    if (candidates.empty()) {
        return false;
    }

    std::sort(candidates.begin(), candidates.end());
    return open_project(candidates.front().string());
}

bool ProjectManager::validate_project(std::string& error_message) const {
    if (!m_project_open) {
        error_message = "No project open";
        return false;
    }

    // Check for missing assets
    auto missing_assets = get_missing_assets();
    if (!missing_assets.empty()) {
        error_message = "Missing assets: " + missing_assets[0];
        return false;
    }

    // Check for missing plugins
    auto missing_plugins = get_missing_plugins();
    if (!missing_plugins.empty()) {
        error_message = "Missing plugins: " + missing_plugins[0];
        return false;
    }

    return true;
}

std::vector<std::string> ProjectManager::get_missing_assets() const {
    std::vector<std::string> missing;
    if (m_project_path.empty() || !fs::exists(m_project_path)) {
        return missing;
    }

    nlohmann::json project_json;
    if (!read_json_file(m_project_path, project_json)) {
        missing.push_back(m_project_path);
        return missing;
    }

    if (project_json.contains("external_files") && project_json["external_files"].is_object()) {
        const fs::path project_dir = fs::path(m_project_path).parent_path();
        for (const auto& [key, value] : project_json["external_files"].items()) {
            if (!value.is_string()) {
                missing.push_back("external_files." + key);
                continue;
            }
            const fs::path referenced = project_dir / value.get<std::string>();
            if (!fs::exists(referenced)) {
                missing.push_back(referenced.string());
            }
        }
    }

    return missing;
}

std::vector<std::string> ProjectManager::get_missing_plugins() const {
    std::vector<std::string> missing;
    std::set<std::string> required;
    m_orchestrator.registry().for_each([&](const Component& comp) {
        required.insert(resolve_plugin_for_component_type(
            const_cast<SimulationOrchestrator&>(m_orchestrator),
            std::string(comp.plugin_type()),
            std::string(comp.component_type())));
    });

    const auto& host = const_cast<SimulationOrchestrator&>(m_orchestrator).plugin_host();
    for (const auto& plugin : required) {
        if (!plugin.empty() && !host.get_plugin(plugin)) {
            missing.push_back(plugin);
        }
    }
    return missing;
}

void ProjectManager::save_session_state() {
    // Session state is saved as part of project
    if (m_project_open) {
        mark_modified();
    }
}

void ProjectManager::restore_session_state() {
    // Session state is loaded as part of project
    // Camera position, selection, etc. are restored from m_session
}

void ProjectManager::perform_autosave() {
    if (!m_project_open || !m_autosave_config.enabled) {
        return;
    }

    // Create autosave directory
    auto autosave_dir = get_autosave_directory();
    try {
        fs::create_directories(autosave_dir);
    } catch (const std::exception& e) {
        return;
    }

    // Generate autosave file path
    auto autosave_path = get_autosave_file_path(0);

    // Serialize current state
    nlohmann::json j;
    if (!serialize_to_json(j)) {
        return;
    }

    // Add autosave metadata
    j["_autosave"] = {
        {"timestamp", std::time(nullptr)},
        {"hash", calculate_project_hash()},
        {"project_path", m_project_path}
    };

    // Save autosave
    std::ofstream file(autosave_path);
    if (!file.is_open()) {
        return;
    }

    try {
        file << std::setw(2) << j;
    } catch (const std::exception& e) {
        return;
    }

    m_last_autosave_time = ImGui::GetTime();
    m_last_autosave_hash = calculate_project_hash();
    auto now = std::time(nullptr);
    char buf[100];
    std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", std::gmtime(&now));
    m_stats.last_autosave_time = buf;

    // Cleanup old autosaves
    cleanup_old_autosaves();

    if (m_on_autosave) {
        m_on_autosave(autosave_path.string());
    }
}

fs::path ProjectManager::get_autosave_directory() const {
    if (m_project_open) {
        fs::path project_dir = get_project_directory();
        return project_dir / m_autosave_config.autosave_dir;
    } else {
        return fs::path(m_autosave_config.autosave_dir);
    }
}

fs::path ProjectManager::get_autosave_file_path(int index) const {
    auto autosave_dir = get_autosave_directory();
    std::string filename = "autosave_" + std::to_string(index) + ".json";
    return autosave_dir / filename;
}

std::vector<fs::path> ProjectManager::get_existing_autosaves() const {
    std::vector<fs::path> autosaves;

    auto autosave_dir = get_autosave_directory();
    if (!fs::exists(autosave_dir)) {
        return autosaves;
    }

    for (const auto& entry : fs::directory_iterator(autosave_dir)) {
        if (entry.path().extension() == ".json" &&
            entry.path().filename().string().find("autosave_") == 0) {
            autosaves.push_back(entry.path());
        }
    }

    // Sort by modification time (newest first)
    std::sort(autosaves.begin(), autosaves.end(),
              [](const fs::path& a, const fs::path& b) {
                  return fs::last_write_time(a) > fs::last_write_time(b);
              });

    return autosaves;
}

void ProjectManager::cleanup_old_autosaves() {
    auto autosaves = get_existing_autosaves();

    // Remove excess autosaves
    while (autosaves.size() > static_cast<size_t>(m_autosave_config.max_autosaves)) {
        fs::remove(autosaves.back());
        autosaves.pop_back();
    }
}

bool ProjectManager::load_autosave_metadata(const fs::path& autosave_path, nlohmann::json& metadata) {
    if (!fs::exists(autosave_path)) {
        return false;
    }

    std::ifstream file(autosave_path);
    if (!file.is_open()) {
        return false;
    }

    nlohmann::json j;
    try {
        file >> j;
        metadata = j.value("_autosave", nlohmann::json{});
        return true;
    } catch (const std::exception& e) {
        return false;
    }
}

uint64_t ProjectManager::calculate_project_hash() const {
    nlohmann::json snapshot;
    if (!serialize_to_json(snapshot)) {
        size_t comp_count = m_orchestrator.registry().size();
        size_t conn_count = m_orchestrator.get_connections().size();
        uint64_t hash = comp_count;
        hash ^= (conn_count << 16);
        hash ^= (static_cast<uint64_t>(m_modified) << 32);
        return hash;
    }

    const std::string dumped = snapshot.dump();
    return fnv1a64_hash(dumped.data(), dumped.size());
}

std::string ProjectManager::get_project_directory() const {
    if (m_project_path.empty()) {
        return fs::current_path().string();
    }
    return fs::path(m_project_path).parent_path().string();
}

std::string ProjectManager::sanitize_filename(const std::string& name) {
    std::string result = name;
    // Replace invalid characters with underscores
    const std::string invalid = "<>:\"/\\|?*";
    for (char c : invalid) {
        std::replace(result.begin(), result.end(), c, '_');
    }
    // Remove leading/trailing spaces and dots
    size_t start = result.find_first_not_of(" .");
    size_t end = result.find_last_not_of(" .");
    if (start == std::string::npos) {
        return "untitled";
    }
    return result.substr(start, end - start + 1);
}

void ProjectManager::mark_modified(bool modified) {
    if (m_modified != modified) {
        m_modified = modified;
        if (m_on_modification_changed) {
            m_on_modification_changed(modified);
        }
    }
}

void ProjectManager::update_statistics() {
    if (m_project_open) {
        m_stats.component_count = m_orchestrator.registry().size();
        m_stats.connection_count = m_orchestrator.get_connections().size();
        m_stats.asset_count = serialize_assets().value("models", nlohmann::json::array()).size();
    }
}

bool ProjectManager::serialize_to_json(nlohmann::json& j, bool split_project_files) const {
    try {
        j["version"] = PROJECT_FORMAT_VERSION;
        j["format"] = PROJECT_FORMAT;

        // Metadata
        j["metadata"] = {
            {"name", m_metadata.name},
            {"description", m_metadata.description},
            {"author", m_metadata.author},
            {"created", m_metadata.created},
            {"modified", m_metadata.modified},
            {"tags", m_metadata.tags}
        };

        // Simulation settings
        j["simulation"] = serialize_simulation_settings();

        // Session state
        j["session"] = {
            {"camera", {
                {"position", {m_session.camera.position.x, m_session.camera.position.y, m_session.camera.position.z}},
                {"target", {m_session.camera.target.x, m_session.camera.target.y, m_session.camera.target.z}},
                {"up", {m_session.camera.up.x, m_session.camera.up.y, m_session.camera.up.z}}
            }},
            {"selected_component", m_session.selected_component},
            {"ui_layout", m_session.ui_layout},
            {"viewport", {
                {"wireframe", m_session.viewport.wireframe},
                {"shading_mode", m_session.viewport.shading_mode},
                {"show_grid", m_session.viewport.show_grid},
                {"grid_size", m_session.viewport.grid_size},
                {"background_color", {m_session.viewport.background_color.x,
                                    m_session.viewport.background_color.y,
                                    m_session.viewport.background_color.z}}
            }}
        };

        nlohmann::json application_state = nlohmann::json::object();
        if (m_on_serialize_application_state) {
            application_state = m_on_serialize_application_state();
        }

        nlohmann::json circuit_json = {
            {"version", 1},
            {"components", serialize_components()},
            {"connections", serialize_connections()},
            {"circuit_mappings", serialize_circuit_mappings()}
        };
        if (application_state.contains("circuit_editor")) {
            circuit_json["circuit_editor"] = application_state["circuit_editor"];
        }

        nlohmann::json models_json = {
            {"version", 1},
            {"assets", serialize_assets()}
        };
        if (application_state.contains("model_editor")) {
            models_json["model_editor"] = application_state["model_editor"];
        }

        nlohmann::json code_json = {
            {"version", 1}
        };
        if (application_state.contains("code_editor")) {
            code_json["code_editor"] = application_state["code_editor"];
        }

        if (split_project_files && !m_project_path.empty()) {
            const fs::path project_dir = fs::path(m_project_path).parent_path();

            bool wrote_sketch_file = false;
            if (code_json.contains("code_editor") &&
                code_json["code_editor"].is_object() &&
                code_json["code_editor"].contains("content")) {
                auto& editor_state = code_json["code_editor"];
                const std::string content = code_json["code_editor"].value("content", std::string{});
                const std::string current_file = editor_state.value("current_file", std::string{});

                fs::path content_path;
                fs::path content_relative_path;
                bool should_write_content = false;

                if (current_file.empty()) {
                    content_path = project_dir / PROJECT_SKETCH_FILE;
                    content_relative_path = fs::path(PROJECT_SKETCH_FILE);
                    editor_state["current_file"] = content_path.string();
                    should_write_content = true;
                } else {
                    content_path = fs::path(current_file);
                    if (content_path.is_relative()) {
                        content_path = project_dir / content_path;
                    }

                    if (project_relative_path(project_dir, content_path, content_relative_path)) {
                        should_write_content = true;
                    }
                }

                if (should_write_content) {
                    if (!write_text_file(content_path, content)) {
                        return false;
                    }
                    wrote_sketch_file = same_normalized_path(content_path, project_dir / PROJECT_SKETCH_FILE);
                }

                editor_state.erase("content");
                if (!content_relative_path.empty()) {
                    editor_state["content_file"] = content_relative_path.generic_string();
                } else {
                    editor_state.erase("content_file");
                }
            }

            if (!write_json_file(project_dir / PROJECT_CIRCUIT_FILE, circuit_json)) return false;
            if (!write_json_file(project_dir / PROJECT_MODELS_FILE, models_json)) return false;
            if (!write_json_file(project_dir / PROJECT_CODE_STATE_FILE, code_json)) return false;

            j["external_files"] = {
                {"circuit", PROJECT_CIRCUIT_FILE},
                {"models", PROJECT_MODELS_FILE},
                {"code_state", PROJECT_CODE_STATE_FILE}
            };
            if (wrote_sketch_file) {
                j["external_files"]["sketch"] = PROJECT_SKETCH_FILE;
            }
        } else {
            // Inline payload is kept for autosaves and legacy single-file exports.
            j["components"] = circuit_json["components"];
            j["connections"] = circuit_json["connections"];
            j["assets"] = models_json["assets"];
            j["circuit_mappings"] = circuit_json["circuit_mappings"];
            j["application_state"] = application_state;
        }

        return true;
    } catch (const std::exception& e) {
        spdlog::error("Failed to serialize project: {}", e.what());
        return false;
    }
}

bool ProjectManager::deserialize_from_json(const nlohmann::json& j) {
    try {
        const fs::path project_dir = m_project_path.empty()
            ? fs::current_path()
            : fs::path(m_project_path).parent_path();

        nlohmann::json circuit_json = j;
        nlohmann::json models_json = j;
        nlohmann::json code_json = nlohmann::json::object();
        bool has_external_code = false;

        if (j.contains("external_files") && j["external_files"].is_object()) {
            const auto& files = j["external_files"];
            nlohmann::json loaded;

            const std::string circuit_path = files.value("circuit", std::string{});
            if (!circuit_path.empty() && read_json_file(project_dir / circuit_path, loaded)) {
                circuit_json = loaded;
            }

            loaded = nlohmann::json{};
            const std::string models_path = files.value("models", std::string{});
            if (!models_path.empty() && read_json_file(project_dir / models_path, loaded)) {
                models_json = loaded;
            }

            loaded = nlohmann::json{};
            const std::string code_path = files.value("code_state", std::string{});
            if (!code_path.empty() && read_json_file(project_dir / code_path, loaded)) {
                code_json = loaded;
                has_external_code = true;

                if (code_json.contains("code_editor") &&
                    code_json["code_editor"].is_object() &&
                    code_json["code_editor"].contains("content_file")) {
                    std::string content;
                    const std::string content_file = code_json["code_editor"].value("content_file", std::string{});
                    fs::path content_path;
                    if (content_file == fs::path(PROJECT_SKETCH_FILE).filename().string()) {
                        // Backward compatibility with older sidecars that stored only "sketch.ino".
                        content_path = project_dir / "code" / content_file;
                    } else {
                        content_path = project_dir / fs::path(content_file);
                    }
                    if (!content_file.empty() && read_text_file(content_path, content)) {
                        code_json["code_editor"]["content"] = content;
                    }
                }
            }
        }

        // Metadata
        if (j.contains("metadata")) {
            auto meta = j["metadata"];
            m_metadata.name = meta.value("name", "Untitled");
            m_metadata.description = meta.value("description", "");
            m_metadata.author = meta.value("author", "");
            m_metadata.created = meta.value("created", "");
            m_metadata.modified = meta.value("modified", "");
            m_metadata.tags = meta.value("tags", std::vector<std::string>{});
        }

        // Components
        if (circuit_json.contains("components") && !deserialize_components(circuit_json["components"])) {
            return false;
        }

        // Connections
        if (circuit_json.contains("connections") && !deserialize_connections(circuit_json["connections"])) {
            return false;
        }

        // Assets
        if (models_json.contains("assets")) {
            deserialize_assets(models_json["assets"]);
        }

        // Circuit mappings
        if (circuit_json.contains("circuit_mappings")) {
            deserialize_circuit_mappings(circuit_json["circuit_mappings"]);
        }

        // Simulation settings
        if (j.contains("simulation")) {
            deserialize_simulation_settings(j["simulation"]);
        }

        // Session state
        if (j.contains("session")) {
            auto session = j["session"];

            if (session.contains("camera")) {
                auto cam = session["camera"];
                if (cam.contains("position")) {
                    auto pos = cam["position"];
                    m_session.camera.position = Vec3{pos[0], pos[1], pos[2]};
                }
                if (cam.contains("target")) {
                    auto tgt = cam["target"];
                    m_session.camera.target = Vec3{tgt[0], tgt[1], tgt[2]};
                }
                if (cam.contains("up")) {
                    auto up = cam["up"];
                    m_session.camera.up = Vec3{up[0], up[1], up[2]};
                }
            }

            m_session.selected_component = session.value("selected_component", "");
            m_session.ui_layout = session.value("ui_layout", nlohmann::json{});

            if (session.contains("viewport")) {
                auto vp = session["viewport"];
                m_session.viewport.wireframe = vp.value("wireframe", false);
                m_session.viewport.shading_mode = vp.value("shading_mode", 2);
                m_session.viewport.show_grid = vp.value("show_grid", true);
                m_session.viewport.grid_size = vp.value("grid_size", 10.0f);
                if (vp.contains("background_color")) {
                    auto bg = vp["background_color"];
                    m_session.viewport.background_color = Vec3{bg[0], bg[1], bg[2]};
                }
            }
        }

        if (m_on_deserialize_application_state) {
            nlohmann::json application_state = j.value("application_state", nlohmann::json::object());
            if (!application_state.is_object()) {
                application_state = nlohmann::json::object();
            }
            if (circuit_json.contains("circuit_editor")) {
                application_state["circuit_editor"] = circuit_json["circuit_editor"];
            }
            if (models_json.contains("model_editor")) {
                application_state["model_editor"] = models_json["model_editor"];
            }
            if (has_external_code && code_json.contains("code_editor")) {
                application_state["code_editor"] = code_json["code_editor"];
            }
            m_on_deserialize_application_state(application_state);
        }

        return true;
    } catch (const std::exception& e) {
        spdlog::error("Failed to deserialize project: {}", e.what());
        return false;
    }
}

nlohmann::json ProjectManager::serialize_components() const {
    nlohmann::json j = nlohmann::json::array();

    m_orchestrator.registry().for_each([this, &j](const Component& comp) {
        nlohmann::json comp_json;

        // Basic component info
        comp_json["id"] = comp.id();
        comp_json["type"] = std::string(comp.component_type());
        comp_json["plugin"] = resolve_plugin_for_component_type(
            m_orchestrator,
            std::string(comp.plugin_type()),
            std::string(comp.component_type()));

        // Transform
        const auto& transform = comp.transform();
        comp_json["transform"] = {
            {"position", {transform.position.x, transform.position.y, transform.position.z}},
            {"rotation", {transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w}},
            {"scale", {transform.scale.x, transform.scale.y, transform.scale.z}}
        };

        // Component-specific parameters
        nlohmann::json params;
        const_cast<Component&>(comp).serialize(params);
        comp_json["parameters"] = params;

        j.push_back(comp_json);
    });

    return j;
}

bool ProjectManager::deserialize_components(const nlohmann::json& j) {
    if (!j.is_array()) {
        return false;
    }

    // Clear existing components
    // Note: In production, should be smarter about this to avoid losing unsaved work

    for (const auto& comp_json : j) {
        std::string id = comp_json.value("id", "");
        std::string type = comp_json.value("type", "");
        std::string plugin = comp_json.value("plugin", "");

        if (id.empty() || type.empty() || plugin.empty()) {
            continue;
        }
        if (m_orchestrator.registry().get(id)) {
            spdlog::warn("Skipping duplicate project component id '{}'", id);
            continue;
        }

        const std::string resolved_plugin = resolve_plugin_for_component_type(m_orchestrator, plugin, type);
        if (resolved_plugin.empty()) {
            spdlog::warn("Project component '{}' ({}) has no matching plugin", id, type);
            continue;
        }

        // Create component. Older project files sometimes stored category names
        // such as "power" here; resolve_plugin_for_component_type maps those
        // back to real plugin IDs like "elec_power".
        auto* comp = m_orchestrator.create_component(resolved_plugin, type, id);
        if (!comp) {
            spdlog::warn("Failed to restore project component '{}' using plugin '{}' and type '{}'",
                         id, resolved_plugin, type);
            continue;
        }

        // Set transform
        if (comp_json.contains("transform")) {
            auto trans = comp_json["transform"];
            if (trans.contains("position")) {
                auto pos = trans["position"];
                comp->transform().position = Vec3{pos[0], pos[1], pos[2]};
            }
            if (trans.contains("rotation")) {
                auto rot = trans["rotation"];
                comp->transform().rotation = Quat{rot[0], rot[1], rot[2], rot[3]};
            }
            if (trans.contains("scale")) {
                auto scl = trans["scale"];
                comp->transform().scale = Vec3{scl[0], scl[1], scl[2]};
            }
        }

        // Set parameters
        if (comp_json.contains("parameters")) {
            comp->deserialize(comp_json["parameters"]);
        }
    }

    return true;
}

nlohmann::json ProjectManager::serialize_connections() const {
    nlohmann::json j = nlohmann::json::array();

    for (const auto& conn : m_orchestrator.get_connections()) {
        if (!conn || !conn->source || !conn->target) {
            continue;
        }

        // A port's owner can be null when the owning component failed to be
        // restored from a project file (e.g. plugin/type missing). Skip such
        // dangling connections instead of dereferencing a null owner, which
        // would crash the auto-save path with SIGSEGV.
        if (!conn->source->owner() || !conn->target->owner()) {
            spdlog::warn("[ProjectManager] skipping connection '{}' with null owner (port source/target owner missing)", conn->uid);
            continue;
        }

        nlohmann::json conn_json;
        conn_json["id"] = conn->uid;
        conn_json["source"] = {
            {"component", conn->source->owner()->id()},
            {"port", std::string(conn->source->name())}
        };
        conn_json["target"] = {
            {"component", conn->target->owner()->id()},
            {"port", std::string(conn->target->name())}
        };

        j.push_back(conn_json);
    }

    return j;
}

bool ProjectManager::deserialize_connections(const nlohmann::json& j) {
    if (!j.is_array()) {
        return false;
    }

    // Clear existing connections
    // Note: In production, should be smarter about this

    for (const auto& conn_json : j) {
        std::string source_comp = conn_json["source"]["component"];
        std::string source_port = conn_json["source"]["port"];
        std::string target_comp = conn_json["target"]["component"];
        std::string target_port = conn_json["target"]["port"];
        std::string uid = conn_json.value("id", "");

        // Find components
        auto* src_comp = m_orchestrator.registry().get(source_comp);
        auto* tgt_comp = m_orchestrator.registry().get(target_comp);

        if (!src_comp || !tgt_comp) {
            continue;
        }

        // Find ports
        Port* src_port = nullptr;
        Port* tgt_port = nullptr;

        for (auto* p : src_comp->get_ports()) {
            if (p && p->name() == source_port) {
                src_port = p;
                break;
            }
        }

        for (auto* p : tgt_comp->get_ports()) {
            if (p && p->name() == target_port) {
                tgt_port = p;
                break;
            }
        }

        if (src_port && tgt_port) {
            m_orchestrator.connect(src_port, tgt_port, uid);
        }
    }

    return true;
}

nlohmann::json ProjectManager::serialize_assets() const {
    nlohmann::json j;
    j["models"] = nlohmann::json::array();
    j["symbols"] = nlohmann::json::array();
    j["plugins"] = nlohmann::json::array();

    std::set<std::string> plugins;
    m_orchestrator.registry().for_each([this, &plugins](const Component& comp) {
        plugins.insert(resolve_plugin_for_component_type(
            m_orchestrator,
            std::string(comp.plugin_type()),
            std::string(comp.component_type())));
    });
    for (const auto& plugin : plugins) {
        if (!plugin.empty()) j["plugins"].push_back(plugin);
    }

    return j;
}

bool ProjectManager::deserialize_assets(const nlohmann::json& j) {
    if (!j.is_object()) return false;
    return true;
}

nlohmann::json ProjectManager::serialize_circuit_mappings() const {
    nlohmann::json j = nlohmann::json::array();

    // TODO: Serialize circuit-actuator mappings

    return j;
}

bool ProjectManager::deserialize_circuit_mappings(const nlohmann::json& j) {
    // TODO: Deserialize circuit-actuator mappings
    return true;
}

nlohmann::json ProjectManager::serialize_simulation_settings() const {
    nlohmann::json j;

    // Time settings
    const auto& time = m_orchestrator.time_manager();
    j["timestep_ms"] = time.physics_step_size() * 1000.0;
    j["realtime_factor"] = time.realtime_factor();
    j["deterministic"] = time.is_deterministic();

    // Physics settings
    // TODO: Serialize physics settings

    // Circuit settings
    // TODO: Serialize circuit settings

    return j;
}

bool ProjectManager::deserialize_simulation_settings(const nlohmann::json& j) {
    // Time settings
    if (j.contains("timestep_ms")) {
        double dt_ms = j["timestep_ms"];
        // Note: TimeManager doesn't have set_physics_step_size, so we skip this for now
        // m_orchestrator.time_manager().set_physics_step_size(dt_ms / 1000.0);
    }

    if (j.contains("realtime_factor")) {
        double factor = j["realtime_factor"];
        m_orchestrator.time_manager().set_realtime_factor(factor);
    }

    if (j.contains("deterministic")) {
        bool det = j["deterministic"];
        m_orchestrator.time_manager().set_deterministic(det);
    }

    // TODO: Deserialize physics and circuit settings

    return true;
}

} // namespace mechatron
