cmake_minimum_required(VERSION 3.20)

project(RoRVehicleCore LANGUAGES CXX)

add_library(ror_vehicle_core STATIC
    DriftDemoRuntime.cpp
    RigPhysicsBuilder.cpp
    ../../../source/main/physics/BeamPhysics.cpp
    ../../../source/main/physics/Differentials.cpp
    ../../../source/main/physics/HydroPhysics.cpp
    ../../../source/main/physics/WheelPhysics.cpp
    ../../../source/main/physics/collision/GroundContact.cpp
    ../../../source/main/physics/collision/GroundFriction.cpp
    ../../../source/main/resources/rig_def_fileformat/PortableRigDef.cpp
)

target_include_directories(ror_vehicle_core
    PUBLIC
        .
        ../../../source/main/physics
        ../../../source/main/physics/collision
        ../../../source/main/resources/rig_def_fileformat
)

target_compile_features(ror_vehicle_core PUBLIC cxx_std_11)
target_compile_definitions(ror_vehicle_core PRIVATE ROR_PORTABLE_CORE=1)
set_target_properties(ror_vehicle_core PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)

include(CTest)
if(BUILD_TESTING AND NOT CMAKE_SYSTEM_NAME STREQUAL "iOS")
    add_executable(ror_vehicle_core_probe
        tests/VehicleCoreProbe.cpp
    )
    target_link_libraries(ror_vehicle_core_probe PRIVATE ror_vehicle_core)
    target_compile_features(ror_vehicle_core_probe PRIVATE cxx_std_11)

    add_executable(ror_chassis_probe
        tests/ChassisProbe.cpp
    )
    target_link_libraries(ror_chassis_probe PRIVATE ror_vehicle_core)
    target_compile_features(ror_chassis_probe PRIVATE cxx_std_11)

    add_executable(ror_wheel_probe
        tests/WheelProbe.cpp
    )
    target_link_libraries(ror_wheel_probe PRIVATE ror_vehicle_core)
    target_compile_features(ror_wheel_probe PRIVATE cxx_std_11)

    add_executable(ror_driven_axle_probe
        tests/DrivenAxleProbe.cpp
    )
    target_link_libraries(ror_driven_axle_probe PRIVATE ror_vehicle_core)
    target_compile_features(ror_driven_axle_probe PRIVATE cxx_std_11)

    add_executable(ror_steering_suspension_probe
        tests/SteeringSuspensionProbe.cpp
    )
    target_link_libraries(ror_steering_suspension_probe PRIVATE ror_vehicle_core)
    target_compile_features(ror_steering_suspension_probe PRIVATE cxx_std_11)

    add_executable(ror_rigdef_probe
        tests/RigDefProbe.cpp
    )
    target_link_libraries(ror_rigdef_probe PRIVATE ror_vehicle_core)
    target_compile_features(ror_rigdef_probe PRIVATE cxx_std_11)

    add_executable(ror_rig_physics_builder_probe
        tests/RigPhysicsBuilderProbe.cpp
    )
    target_link_libraries(ror_rig_physics_builder_probe PRIVATE ror_vehicle_core)
    target_compile_features(ror_rig_physics_builder_probe PRIVATE cxx_std_11)

    add_executable(ror_drift_demo_runtime_probe
        tests/DriftDemoRuntimeProbe.cpp
    )
    target_link_libraries(ror_drift_demo_runtime_probe PRIVATE ror_vehicle_core)
    target_compile_features(ror_drift_demo_runtime_probe PRIVATE cxx_std_11)

    add_test(
        NAME ror_vehicle_core_probe
        COMMAND ror_vehicle_core_probe
    )
    add_test(
        NAME ror_chassis_probe
        COMMAND ror_chassis_probe
    )
    add_test(
        NAME ror_wheel_probe
        COMMAND ror_wheel_probe
    )
    add_test(
        NAME ror_driven_axle_probe
        COMMAND ror_driven_axle_probe
    )
    add_test(
        NAME ror_steering_suspension_probe
        COMMAND ror_steering_suspension_probe
    )
    add_test(
        NAME ror_rigdef_probe
        COMMAND ror_rigdef_probe
    )
    add_test(
        NAME ror_rig_physics_builder_probe
        COMMAND ror_rig_physics_builder_probe
    )
    add_test(
        NAME ror_drift_demo_runtime_probe
        COMMAND ror_drift_demo_runtime_probe
    )
endif()
