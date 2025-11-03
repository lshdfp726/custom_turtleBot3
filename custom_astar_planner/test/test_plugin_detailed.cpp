#include <iostream>
#include <pluginlib/class_loader.hpp>
#include "nav2_core/global_planner.hpp"

int main() {
    std::cout << "=== Detailed Plugin Test ===" << std::endl;
    
    try {
        // 创建类加载器
        pluginlib::ClassLoader<nav2_core::GlobalPlanner> loader(
            "nav2_core", "nav2_core::GlobalPlanner");
        
        std::cout << "✅ ClassLoader created" << std::endl;
        
        // 获取所有可用的插件
        auto plugins = loader.getDeclaredClasses();
        std::cout << "Found " << plugins.size() << " GlobalPlanner plugins:" << std::endl;
        for (const auto& plugin : plugins) {
            std::cout << "  - " << plugin << std::endl;
        }
        
        // 特别检查你的插件
        std::string your_plugin = "custom_astar_planner/AStarPlanner";
        std::cout << "\n=== Checking: " << your_plugin << " ===" << std::endl;
        
        if (loader.isClassAvailable(your_plugin)) {
            std::cout << "✅ Plugin is available" << std::endl;
            
            try {
                auto planner = loader.createSharedInstance(your_plugin);
                std::cout << "✅ Plugin instance created successfully!" << std::endl;
                std::cout << "🎉 SUCCESS: Your plugin is fully functional!" << std::endl;
            } catch (const std::exception& e) {
                std::cout << "❌ Failed to create instance: " << e.what() << std::endl;
                std::cout << "This indicates a problem with your plugin's constructor or dependencies" << std::endl;
            }
        } else {
            std::cout << "❌ Plugin is NOT available" << std::endl;
            std::cout << "This means the plugin is not properly registered in the system" << std::endl;
        }
        
    } catch (const std::exception& e) {
        std::cerr << "❌ ERROR: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}