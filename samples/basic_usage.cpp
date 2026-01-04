// basic_usage.cpp - HFSM v1.0 Basic Example
#include <hfsm/hfsm_builder.hpp>
#include <iostream>
#include <thread>
#include <chrono>

int main() {
    std::cout << "=== HFSM v1.0 Basic Example ===" << std::endl;
    
    try {
        // 1. Create a minimal HFSM
        auto fsm = ns_fsm::builders::createMinimalHFSM()
            .buildAndInitialize();
        
        std::cout << "1. HFSM initialized. State: " 
                  << fsm->getCurrentStateName() << std::endl;
        
        // 2. Wait for initialization to complete
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        std::cout << "2. Current state: " 
                  << fsm->getCurrentStateName() << std::endl;
        
        // 3. Get statistics
        auto stats = fsm->getStatistics();
        std::cout << "3. Statistics - Loops: " 
                  << stats["loops"].get<uint64_t>() << std::endl;
        
        // 4. Graceful shutdown
        std::cout << "4. Shutting down..." << std::endl;
        fsm->shutdown();
        
        std::cout << "✅ Example completed successfully!" << std::endl;
        return 0;
        
    } catch (const std::exception& e) {
        std::cerr << "❌ Error: " << e.what() << std::endl;
        return 1;
    }
}
