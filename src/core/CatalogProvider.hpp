#pragma once

#include <filesystem>
#include <nlohmann/json.hpp>
#include <optional>
#include <mutex>
#include <string>
#include <chrono>

namespace mechatron {

class CatalogProvider {
public:
    virtual ~CatalogProvider() = default;
    virtual std::optional<nlohmann::json> load_catalog() const = 0;
};

class LocalManifestCatalogProvider final : public CatalogProvider {
public:
    explicit LocalManifestCatalogProvider(std::optional<std::filesystem::path> manifest_path = std::nullopt);

    std::optional<nlohmann::json> load_catalog() const override;

    static std::optional<std::filesystem::path> find_default_catalog_path();

private:
    std::optional<std::filesystem::path> m_manifest_path;
    mutable std::mutex m_cache_mutex;
    mutable std::optional<std::filesystem::path> m_cached_path;
    mutable std::optional<std::filesystem::file_time_type> m_cached_mtime;
    mutable std::optional<nlohmann::json> m_cached_catalog;
};

class BackendApiCatalogProvider final : public CatalogProvider {
public:
    explicit BackendApiCatalogProvider(std::string base_url = {});

    std::optional<nlohmann::json> load_catalog() const override;

private:
    std::string m_base_url;
    mutable std::mutex m_cache_mutex;
    mutable std::optional<nlohmann::json> m_cached_catalog;
    mutable std::optional<std::chrono::steady_clock::time_point> m_last_success_at;
    mutable std::optional<std::chrono::steady_clock::time_point> m_last_failure_at;
    mutable std::string m_last_failure_reason;
};

const CatalogProvider& default_catalog_provider();

} // namespace mechatron
