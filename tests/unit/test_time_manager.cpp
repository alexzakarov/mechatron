#include <gtest/gtest.h>
#include "TimeManager.hpp"

using namespace mechatron;

TEST(TimeManager, InitialState) {
    TimeManager tm;
    EXPECT_EQ(tm.state(), SimulationState::Stopped);
    EXPECT_DOUBLE_EQ(tm.simulation_time(), 0.0);
    EXPECT_EQ(tm.current_tick(), 0u);
}

TEST(TimeManager, StartSetsRunning) {
    TimeManager tm;
    tm.start();
    EXPECT_EQ(tm.state(), SimulationState::Running);
}

TEST(TimeManager, PauseStopsRunning) {
    TimeManager tm;
    tm.start();
    tm.pause();
    EXPECT_EQ(tm.state(), SimulationState::Paused);
}

TEST(TimeManager, PausePreventsTickAdvance) {
    TimeManager tm;
    tm.start();
    tm.pause();

    auto tick_before = tm.current_tick();
    auto time_before = tm.simulation_time();
    tm.update();

    EXPECT_EQ(tm.current_tick(), tick_before);
    EXPECT_DOUBLE_EQ(tm.simulation_time(), time_before);
}

TEST(TimeManager, ResumeRestoresRunning) {
    TimeManager tm;
    tm.start();
    tm.pause();
    tm.resume();
    EXPECT_EQ(tm.state(), SimulationState::Running);
}

TEST(TimeManager, StopResetsState) {
    TimeManager tm;
    tm.start();
    tm.stop();
    EXPECT_EQ(tm.state(), SimulationState::Stopped);
    EXPECT_DOUBLE_EQ(tm.simulation_time(), 0.0);
    EXPECT_EQ(tm.current_tick(), 0u);
}

TEST(TimeManager, SingleStep) {
    TimeManager tm;
    tm.step();
    EXPECT_EQ(tm.state(), SimulationState::Paused);
    EXPECT_EQ(tm.current_tick(), 1u);
    EXPECT_DOUBLE_EQ(tm.simulation_time(), tm.physics_step_size());
    EXPECT_TRUE(tm.consume_step_request());
    EXPECT_FALSE(tm.consume_step_request());
}

TEST(TimeManager, MultipleSteps) {
    TimeManager tm;
    for (int i = 0; i < 5; ++i) tm.step();
    EXPECT_EQ(tm.current_tick(), 5u);
}

TEST(TimeManager, RealtimeFactor) {
    TimeManager tm;
    tm.set_realtime_factor(2.0);
    EXPECT_DOUBLE_EQ(tm.realtime_factor(), 2.0);
}

TEST(TimeManager, DeterministicMode) {
    TimeManager tm;
    EXPECT_TRUE(tm.is_deterministic());
    tm.set_deterministic(false);
    EXPECT_FALSE(tm.is_deterministic());
}
