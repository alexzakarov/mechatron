#include "EventBus.hpp"

namespace mechatron {

// Subscription move semantics
Subscription::Subscription(Subscription&& other) noexcept
    : m_id(other.m_id), m_bus(other.m_bus) {
    other.m_id = 0;
    other.m_bus = nullptr;
}

Subscription& Subscription::operator=(Subscription&& other) noexcept {
    if (this != &other) {
        unsubscribe();
        m_id = other.m_id;
        m_bus = other.m_bus;
        other.m_id = 0;
        other.m_bus = nullptr;
    }
    return *this;
}

Subscription::~Subscription() {
    unsubscribe();
}

void Subscription::unsubscribe() {
    if (m_bus && m_id != 0) {
        m_bus->unsubscribe(m_id);
        m_id = 0;
        m_bus = nullptr;
    }
}

// EventBus implementation
Subscription EventBus::subscribe(EventType type, EventHandler handler) {
    uint64_t id = m_next_id++;
    m_subscriptions.push_back({id, type, EventDomain::System, "", std::move(handler), true});
    return {id, this};
}

Subscription EventBus::subscribe(std::string_view source_id, EventHandler handler) {
    uint64_t id = m_next_id++;
    // Use EventType count as wildcard for source-based subscriptions
    m_subscriptions.push_back({id, EventType::TemperatureChanged, EventDomain::System,
        std::string(source_id), std::move(handler), true});
    return {id, this};
}

Subscription EventBus::subscribe_domain(EventDomain domain, EventHandler handler) {
    uint64_t id = m_next_id++;
    m_subscriptions.push_back({id, EventType::SimStart, domain, "", std::move(handler), true});
    return {id, this};
}

void EventBus::publish(const Event& event) {
    for (auto& sub : m_subscriptions) {
        if (!sub.active) continue;

        bool match = false;

        // Match by source_id if subscribed to a specific source
        if (!sub.source_id.empty()) {
            match = (sub.source_id == event.source_id);
        }
        // Match by event type (only if not a source-based subscription)
        else if (sub.type == event.type) {
            match = true;
        }

        if (match) {
            sub.handler(event);
        }
    }
}

void EventBus::unsubscribe(uint64_t subscription_id) {
    for (auto& sub : m_subscriptions) {
        if (sub.id == subscription_id) {
            sub.active = false;
            return;
        }
    }
}

} // namespace mechatron
