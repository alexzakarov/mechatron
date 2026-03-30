#include <gtest/gtest.h>
#include "EventBus.hpp"

using namespace mechatron;

TEST(EventBus, PublishSubscribe) {
    EventBus bus;
    int count = 0;

    auto sub = bus.subscribe(EventType::SimStart, [&](const Event& e) {
        count++;
        EXPECT_EQ(e.type, EventType::SimStart);
    });

    bus.publish(Event{EventType::SimStart, EventDomain::System, "", "", std::monostate{}, 0});
    EXPECT_EQ(count, 1);

    bus.publish(Event{EventType::SimStop, EventDomain::System, "", "", std::monostate{}, 0});
    EXPECT_EQ(count, 1); // Should not increment for different event type
}

TEST(EventBus, SubscriptionRAII) {
    EventBus bus;
    int count = 0;

    {
        auto sub = bus.subscribe(EventType::SimStart, [&](const Event&) { count++; });
        bus.publish(Event{EventType::SimStart, EventDomain::System, "", "", std::monostate{}, 0});
        EXPECT_EQ(count, 1);
    }

    bus.publish(Event{EventType::SimStart, EventDomain::System, "", "", std::monostate{}, 0});
    EXPECT_EQ(count, 1); // Subscription should be gone
}

TEST(EventBus, SourceFiltering) {
    EventBus bus;
    std::string received_source;

    auto sub = bus.subscribe("mcu_1", [&](const Event& e) {
        received_source = e.source_id;
    });

    bus.publish(Event{EventType::PinChanged, EventDomain::Software, "mcu_1", "", std::monostate{}, 0});
    EXPECT_EQ(received_source, "mcu_1");

    bus.publish(Event{EventType::PinChanged, EventDomain::Software, "mcu_2", "", std::monostate{}, 0});
    EXPECT_EQ(received_source, "mcu_1"); // Should not change
}

TEST(EventBus, MultipleSubscribers) {
    EventBus bus;
    int count_a = 0, count_b = 0;

    auto sub_a = bus.subscribe(EventType::SimStep, [&](const Event&) { count_a++; });
    auto sub_b = bus.subscribe(EventType::SimStep, [&](const Event&) { count_b++; });

    bus.publish(Event{EventType::SimStep, EventDomain::System, "", "", std::monostate{}, 0});
    EXPECT_EQ(count_a, 1);
    EXPECT_EQ(count_b, 1);
}

TEST(EventBus, EventTick) {
    EventBus bus;
    uint64_t tick = 0;

    auto sub = bus.subscribe(EventType::SimStep, [&](const Event& e) { tick = e.tick; });
    bus.publish(Event{EventType::SimStep, EventDomain::System, "", "", std::monostate{}, 42});
    EXPECT_EQ(tick, 42u);
}
