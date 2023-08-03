cmake_minimum_required( VERSION 3.10 )

set(CMAKE_CXX_STANDARD 20)

project( controller
        VERSION 2021.1.0
        DESCRIPTION "Cellworld vr utilities"
        LANGUAGES CXX)

####
#### DEPENDENCIES
####

find_package (Dependencies QUIET)

if (NOT ${Dependencies_FOUND})
    if (NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/DependenciesConfig.cmake)
        file(DOWNLOAD https://raw.githubusercontent.com/germanespinosa/dependencies/main/DependenciesConfig.cmake ${CMAKE_CURRENT_BINARY_DIR}/DependenciesConfig.cmake)
    endif()
    set(Dependencies_DIR "${CMAKE_CURRENT_BINARY_DIR}")
    find_package (Dependencies REQUIRED)
endif()

####
#### END DEPENDENCIES
####

string(APPEND CMAKE_CXX_FLAGS " -pthread ")
install_git_dependency(Cellworld https://github.com/germanespinosa/cellworld
        CMAKE_PROJECT
        IMPORT_TARGETS cellworld json-cpp)
install_git_dependency(Easy-tcp https://github.com/germanespinosa/easy-tcp
        CMAKE_PROJECT
        IMPORT_TARGETS easy-tcp)
install_git_dependency(Tcp_messages https://github.com/cellworld-neuro/dependency_tcp-messages
        CMAKE_PROJECT
        IMPORT_TARGETS tcp_messages)
install_git_dependency(Agent_tracking https://github.com/germanespinosa/agent_tracking
        CMAKE_PROJECT
        IMPORT_TARGETS agent_tracking)
install_git_dependency(Experiment_service https://github.com/germanespinosa/experiment_service
        CMAKE_PROJECT
        IMPORT_TARGETS experiment_service)


###
### MAIN LIBRARY SETUP
###

set(controller_files
        src/pid_controller.cpp
        src/controller_service.cpp
        src/controller_client.cpp
        src/agent.cpp)

add_library(controller ${controller_files})

set_target_properties(controller
        PROPERTIES
        CXX_STANDARD 20
        CXX_STANDARD_REQUIRED On
        CXX_EXTENSIONS Off
        VERSION ${PROJECT_VERSION})

target_include_directories(controller
        SYSTEM INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

include_directories(include)

target_link_libraries( controller
        LINK_PUBLIC
        agent_tracking
        experiment_service)

###
### LIBRARY INSTALLATION
###

include(GNUInstallDirs)

install(TARGETS controller EXPORT ControllerConfig
        ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR})

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

install(EXPORT ControllerConfig
        DESTINATION ${CMAKE_INSTALL_DATADIR}/cmake/Controller
        EXPORT_LINK_INTERFACE_LIBRARIES)

export(TARGETS controller FILE ControllerConfig.cmake)
install(CODE "FILE(APPEND ${CMAKE_INSTALL_FULL_DATADIR}/cmake/Controller/ControllerConfig.cmake \"find_package(Params-cpp REQUIRED)\n\") ")
install(CODE "FILE(APPEND ${CMAKE_INSTALL_FULL_DATADIR}/cmake/Controller/ControllerConfig.cmake \"find_package(Cellworld REQUIRED)\n\") ")
install(CODE "FILE(APPEND ${CMAKE_INSTALL_FULL_DATADIR}/cmake/Controller/ControllerConfig.cmake \"set(CONTROLLER_LIBRARIES ${CELLWORLD_LIBRARIES} params-cpp )\n\") ")
