#pragma once

//
// AppConfig — central place for desktop (C++) application defaults.
//
// All hard-coded ports, URLs, and host names should live here. Update a value
// in this single file and every subsystem (CatalogProvider, MCP server, CLI)
// picks it up automatically.
//

namespace mechatron {
namespace app_config {

// Host for the local backend API.
inline constexpr const char* kBackendHost = "127.0.0.1";

// Port for the local backend API (Go server).
inline constexpr int kBackendPort = 8081;

// Path of the catalog bootstrap endpoint on the backend.
inline constexpr const char* kBackendCatalogPath =
    "/api/v1/catalog/bootstrap";

// Default port for the CLI's MCP server.
inline constexpr int kMcpDefaultPort = 8081;

// Environment variable that, when set, overrides the backend base URL.
inline constexpr const char* kBackendUrlEnv = "MECHATRON_BACKEND_URL";

} // namespace app_config
} // namespace mechatron
