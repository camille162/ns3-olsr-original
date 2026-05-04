cmake_minimum_required(VERSION 3.13)
project(AntiDroneSim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(NS3_DIR /home/wm/repos/ns-allinone-3.45/ns-3.45)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/model
    ${CMAKE_CURRENT_SOURCE_DIR}/helper
    ${NS3_DIR}/build/include
)

link_directories(
    ${NS3_DIR}/build/lib
)

add_library(etx-olsr STATIC
    model/etx-olsr-routing-protocol.cc
    model/control-agent.cc
    helper/etx-olsr-helper.cc
    helper/control-agent-helper.cc
)

target_link_libraries(etx-olsr
    ns3.45-core-default
    ns3.45-network-default
    ns3.45-internet-default
    ns3.45-olsr-default
)

add_executable(sim main.cc)

target_link_libraries(sim
    etx-olsr
    ns3.45-core-default
    ns3.45-network-default
    ns3.45-internet-default
    ns3.45-mobility-default
    ns3.45-wifi-default
    ns3.45-energy-default
    ns3.45-spectrum-default
    ns3.45-propagation-default
    ns3.45-flow-monitor-default
    ns3.45-applications-default
    ns3.45-olsr-default
    pthread
)

message(STATUS "✅ AntiDroneSim + ControlAgent 配置完成")
