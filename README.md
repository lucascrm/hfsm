[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![C++17](https://img.shields.io/badge/C++-17-blue.svg)](https://en.cppreference.com/w/cpp/17)
[![Build Status](https://img.shields.io/badge/build-passing-brightgreen.svg)]()

A modern, thread-safe, hierarchical finite state machine library for C++17.

## Features
- 🚀 **Zero dependencies** - Pure C++17 STL
- 🛡️  **Thread-safe** - Designed for concurrent systems
- 🏗️  **Hierarchical states** - Support for state nesting
- 🔌 **Event-driven** - Built-in event system
- 📊 **Comprehensive logging** - Configurable logging levels
- ⚙️ **Dynamic configuration** - Runtime configuration support
- 🧪 **Tested** - Comprehensive test suite
- 🏭 **Builder pattern** - Fluent API for configuration

## Quick Start

```cpp
#include <hfsm/hfsm_builder.hpp>
#include <iostream>

int main() {
    using namespace ns_fsm;

    // Create a pre-configured HFSM
    auto fsm = builders::createRoboticsHFSM()
        .buildAndInitialize();

    std::cout << "HFSM initialized. State: "
              << fsm->getCurrentStateName() << std::endl;

    // Publish events
    Event task_event(EventType::TASK_START, "mission_1");
    fsm->publishEvent(task_event);

    // Graceful shutdown
    fsm->shutdown();

    return 0;
}
