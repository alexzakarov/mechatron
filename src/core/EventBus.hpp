#pragma once

#include <string>
#include <string_view>
#include <functional>
#include <vector>
#include <unordered_map>
#include <memory>
#include <any>
#include <cstdint>

namespace mechatron {

enum class EventDomain {
    Software,
    Electronics,
    Mechanics,
    Multiphysics,
    System
};

enum class EventType : uint32_t {
    // System
    SimStart = 0,
    SimPause,
    SimResume,
    SimStop,
    SimStep,
    PluginLoaded,
    PluginUnloaded,

    // Software
    PinChanged,
    FirmwareLoaded,
    UartData,
    ControlOutput,

    // Electronics
    VoltageChanged,
    CurrentChanged,
    ProtocolFrame,
    PowerStateChanged,

    // Mechanics
    ForceApplied,
    PositionChanged,
    Collision,
    ConstraintBroken,

    // Multiphysics
    TemperatureChanged,
    MagneticFluxChanged,
    PressureChanged,
    VibrationDetected
};

struct Event {
    EventType type;
    EventDomain domain;
    std::string source_id;
    std::string target_id;  // empty = broadcast
    std::any data;
    uint64_t tick;
};

using EventHandler = std::function<void(const Event&)>;

class Subscription {
public:
    Subscription() = default;
    Subscription(uint64_t id, class EventBus* bus) : m_id(id), m_bus(bus) {}
    ~Subscription();

    Subscription(Subscription&& other) noexcept;
    Subscription& operator=(Subscription&& other) noexcept;

    Subscription(const Subscription&) = delete;
    Subscription& operator=(const Subscription&) = delete;

    void unsubscribe();

private:
    uint64_t m_id = 0;
    class EventBus* m_bus = nullptr;
};

class EventBus {
public:
    Subscription subscribe(EventType type, EventHandler handler);
    Subscription subscribe(std::string_view source_id, EventHandler handler);
    Subscription subscribe_domain(EventDomain domain, EventHandler handler);

    void publish(const Event& event);
    void unsubscribe(uint64_t subscription_id);

private:
    struct SubscriptionEntry {
        uint64_t id;
        EventType type;
        EventDomain domain;
        std::string source_id;
        EventHandler handler;
        bool active = true;
    };

    uint64_t m_next_id = 1;
    std::vector<SubscriptionEntry> m_subscriptions;
};

} // namespace mechatron
