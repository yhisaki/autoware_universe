cmake_minimum_required(VERSION 3.14)
project(autoware_diffusion_planner)

find_package(autoware_cmake REQUIRED)
autoware_package()
find_package(Boost REQUIRED)
find_package(onnxruntime QUIET)
find_package(xtensor REQUIRED)

# Options
option(CUDA_VERBOSE "Verbose output of CUDA modules" OFF)

# Find CUDA
option(CUDA_AVAIL "CUDA available" OFF)
find_package(CUDA)
if(CUDA_FOUND)
  find_library(CUBLAS_LIBRARIES cublas HINTS
    ${CUDA_TOOLKIT_ROOT_DIR}/lib64
    ${CUDA_TOOLKIT_ROOT_DIR}/lib
  )
  if(CUDA_VERBOSE)
    message(STATUS "CUDA is available!")
    message(STATUS "CUDA Libs: ${CUDA_LIBRARIES}")
    message(STATUS "CUDA Headers: ${CUDA_INCLUDE_DIRS}")
  endif()
  unset(CUDA_cublas_device_LIBRARY CACHE)
  set(CUDA_AVAIL ON)
else()
  message(WARNING "CUDA NOT FOUND")
  set(CUDA_AVAIL OFF)
endif()

# Find TensorRT libraries
option(TRT_AVAIL "TensorRT available" OFF)
find_library(NVINFER nvinfer)
find_library(NVONNXPARSER nvonnxparser)
if(NVINFER AND NVONNXPARSER)
  if(CUDA_VERBOSE)
    message(STATUS "TensorRT is available!")
    message(STATUS "NVINFER: ${NVINFER}")
    message(STATUS "NVONNXPARSER: ${NVONNXPARSER}")
  endif()
  set(TRT_AVAIL ON)
else()
  message(WARNING "TensorRT is NOT Available")
  set(TRT_AVAIL OFF)
endif()

# Find acados for the trajectory optimization (optional)
option(ACADOS_AVAIL "acados available" OFF)
set(ACADOS_SOURCE_DIR "$ENV{ACADOS_SOURCE_DIR}")
if(NOT ACADOS_SOURCE_DIR)
  set(ACADOS_SOURCE_DIR "/opt/acados")
endif()
find_library(ACADOS_LIBRARY acados HINTS ${ACADOS_SOURCE_DIR}/lib)
find_library(BLASFEO_LIBRARY blasfeo HINTS ${ACADOS_SOURCE_DIR}/lib)
find_library(HPIPM_LIBRARY hpipm HINTS ${ACADOS_SOURCE_DIR}/lib)
set(ACADOS_PYTHON "${ACADOS_SOURCE_DIR}/.venv/bin/python3")
if(ACADOS_LIBRARY AND BLASFEO_LIBRARY AND HPIPM_LIBRARY AND EXISTS "${ACADOS_PYTHON}")
  set(ACADOS_AVAIL ON)
else()
  message(WARNING
    "acados NOT found (ACADOS_SOURCE_DIR=${ACADOS_SOURCE_DIR}); "
    "trajectory optimization support will be disabled")
  set(ACADOS_AVAIL OFF)
endif()

if(TRT_AVAIL AND CUDA_AVAIL)
  if(NOT onnxruntime_FOUND)
    find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
      HINTS /usr/local/include /usr/local/include/onnxruntime
    )
    find_library(ONNXRUNTIME_LIBRARY onnxruntime
      HINTS /usr/local/lib /usr/local/lib64
    )
    if(ONNXRUNTIME_INCLUDE_DIR AND ONNXRUNTIME_LIBRARY)
      set(onnxruntime_FOUND TRUE)
    endif()
  endif()

  if(onnxruntime_FOUND)
    message(STATUS "ONNX Runtime is available; enabling ONNX Runtime inference support")
  else()
    message(WARNING "ONNX Runtime is NOT Available; ONNX Runtime inference support will be disabled")
  endif()

  # Add compile option to suppress deprecation warnings (optional)
  add_compile_options(-Wno-deprecated-declarations)

  # Suppress narrowing conversion warnings from TensorRT headers
  add_compile_options(-Wno-narrowing)

  set(DIFFUSION_PLANNER_SOURCES
    src/diffusion_planner_core.cpp
    src/diffusion_planner_node.cpp
    src/preprocessing/items/agent.cpp
    src/preprocessing/input_builder.cpp
    src/preprocessing/items/ego_history.cpp
    src/preprocessing/items/ego_shape.cpp
    src/preprocessing/items/goal_pose.cpp
    src/preprocessing/items/map.cpp
    src/preprocessing/items/initial_noise.cpp
    src/preprocessing/items/turn_indicators.cpp
    src/postprocessing/postprocessing_utils.cpp
    src/preprocessing/preprocessing_utils.cpp
    src/preprocessing/items/traffic_signals.cpp
    src/utils/utils.cpp
    src/utils/marker_utils.cpp
    src/utils/planning_factor_utils.cpp
    src/inference/single_step_inference.cpp
    src/inference/utils.cpp
  )
  if(onnxruntime_FOUND)
    list(APPEND DIFFUSION_PLANNER_SOURCES
      src/inference/onnxruntime_inference.cpp
    )
  endif()

  if(ACADOS_AVAIL)
    # acados solver code generation (Python -> C, exported to the build tree)
    set(ACADOS_MODEL diffusion_planner_optimizer)
    set(ACADOS_GEN_ROOT "${CMAKE_CURRENT_BINARY_DIR}/acados_generated")
    set(ACADOS_GEN_DIR "${ACADOS_GEN_ROOT}/c_generated_code")
    file(MAKE_DIRECTORY ${ACADOS_GEN_ROOT})
    set(ACADOS_GENERATED_FILES
      ${ACADOS_GEN_DIR}/acados_solver_${ACADOS_MODEL}.h
      ${ACADOS_GEN_DIR}/acados_solver_${ACADOS_MODEL}.c
      ${ACADOS_GEN_DIR}/${ACADOS_MODEL}_model/${ACADOS_MODEL}_expl_ode_fun.c
      ${ACADOS_GEN_DIR}/${ACADOS_MODEL}_model/${ACADOS_MODEL}_expl_vde_forw.c
      ${ACADOS_GEN_DIR}/${ACADOS_MODEL}_model/${ACADOS_MODEL}_expl_vde_adj.c
      ${ACADOS_GEN_DIR}/${ACADOS_MODEL}_constraints/${ACADOS_MODEL}_constr_h_fun.c
      ${ACADOS_GEN_DIR}/${ACADOS_MODEL}_constraints/${ACADOS_MODEL}_constr_h_fun_jac_uxt_zt.c
    )
    set_source_files_properties(${ACADOS_GENERATED_FILES} PROPERTIES GENERATED TRUE)
    add_custom_command(
      OUTPUT ${ACADOS_GENERATED_FILES}
      COMMAND ${CMAKE_COMMAND} -E env
        "ACADOS_SOURCE_DIR=${ACADOS_SOURCE_DIR}"
        "PYTHONPATH=${ACADOS_SOURCE_DIR}/interfaces/acados_template:${CMAKE_CURRENT_SOURCE_DIR}/scripts:$ENV{PYTHONPATH}"
        ${ACADOS_PYTHON} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_solver.py
      WORKING_DIRECTORY ${ACADOS_GEN_ROOT}
      DEPENDS
        ${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_solver.py
        ${CMAKE_CURRENT_SOURCE_DIR}/scripts/vehicle_model.py
      COMMENT "Generating acados trajectory optimization solver"
    )
    add_custom_target(generate_diffusion_planner_acados_code DEPENDS ${ACADOS_GENERATED_FILES})

    add_library(autoware_diffusion_planner_acados SHARED
      src/optimization/acados_solver_wrapper.cpp
      ${ACADOS_GENERATED_FILES}
    )
    add_dependencies(autoware_diffusion_planner_acados generate_diffusion_planner_acados_code)
    target_include_directories(autoware_diffusion_planner_acados
      PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
      PRIVATE
        ${ACADOS_GEN_ROOT}
        ${ACADOS_GEN_DIR}
        ${ACADOS_SOURCE_DIR}/include
        ${ACADOS_SOURCE_DIR}/include/acados
        ${ACADOS_SOURCE_DIR}/include/acados_c
        ${ACADOS_SOURCE_DIR}/include/blasfeo/include
        ${ACADOS_SOURCE_DIR}/include/hpipm/include
    )
    target_link_libraries(autoware_diffusion_planner_acados
      ${ACADOS_LIBRARY}
      ${BLASFEO_LIBRARY}
      ${HPIPM_LIBRARY}
    )
    set_target_properties(autoware_diffusion_planner_acados PROPERTIES
      BUILD_RPATH "${ACADOS_SOURCE_DIR}/lib"
      INSTALL_RPATH "${ACADOS_SOURCE_DIR}/lib"
    )
    if(CMAKE_C_COMPILER_ID STREQUAL "GNU" OR CMAKE_C_COMPILER_ID MATCHES "Clang")
      set_source_files_properties(${ACADOS_GENERATED_FILES} PROPERTIES
        COMPILE_OPTIONS "-Wno-unused-variable;-Wno-unused-parameter;-Wno-unused-function")
    endif()

    list(APPEND DIFFUSION_PLANNER_SOURCES
      src/optimization/trajectory_optimizer.cpp
    )
  endif()

  # Add your component library
  ament_auto_add_library(autoware_diffusion_planner_component SHARED
    ${DIFFUSION_PLANNER_SOURCES}
  )

  target_include_directories(autoware_diffusion_planner_component PUBLIC
    ${CUDA_INCLUDE_DIRS}
    ${autoware_cuda_utils_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
  )
  if(onnxruntime_FOUND)
    target_include_directories(autoware_diffusion_planner_component PUBLIC
      ${ONNXRUNTIME_INCLUDE_DIR}
    )
    target_compile_definitions(autoware_diffusion_planner_component PUBLIC
      AUTOWARE_DIFFUSION_PLANNER_USE_ONNXRUNTIME
    )
  endif()
  target_link_libraries(autoware_diffusion_planner_component
    ${NVINFER}
    ${NVONNXPARSER}
    ${CUDA_LIBRARIES}
    ${CUBLAS_LIBRARIES}
    xtensor
  )

  if(ACADOS_AVAIL)
    target_link_libraries(autoware_diffusion_planner_component autoware_diffusion_planner_acados)
    target_compile_definitions(autoware_diffusion_planner_component PUBLIC
      AUTOWARE_DIFFUSION_PLANNER_USE_ACADOS
    )
    install(TARGETS autoware_diffusion_planner_acados
      ARCHIVE DESTINATION lib
      LIBRARY DESTINATION lib
      RUNTIME DESTINATION bin
    )
  endif()

  if(onnxruntime_FOUND)
    if(TARGET onnxruntime::onnxruntime)
      target_link_libraries(autoware_diffusion_planner_component onnxruntime::onnxruntime)
    elseif(TARGET onnxruntime)
      target_link_libraries(autoware_diffusion_planner_component onnxruntime)
    else()
      target_link_libraries(autoware_diffusion_planner_component ${ONNXRUNTIME_LIBRARY})
    endif()
  endif()

  autoware_agnocast_wrapper_register_node(autoware_diffusion_planner_component
    PLUGIN "autoware::diffusion_planner::DiffusionPlanner"
    EXECUTABLE autoware_diffusion_planner_node
    ROS2_EXECUTOR SingleThreadedExecutor
    AGNOCAST_EXECUTOR CallbackIsolatedAgnocastExecutor
  )

  if(BUILD_TESTING)
    set(DIFFUSION_PLANNER_TEST_SOURCES
      test/agent_test.cpp
      test/lanelet_test.cpp
      test/lane_segments_test.cpp
      test/pre_processing_utils_test.cpp
      test/postprocessing_utils_test.cpp
      test/utils_test.cpp
      test/marker_utils_test.cpp
      test/lanelet_integration_test.cpp
      test/postprocessing_utils_edge_case_test.cpp
      test/preprocessing_utils_edge_case_test.cpp
      test/agent_edge_case_test.cpp
      test/lanelet_edge_case_test.cpp
      test/diffusion_planner_integration_test.cpp
      test/planning_factor_utils_test.cpp)
    if(ACADOS_AVAIL)
      list(APPEND DIFFUSION_PLANNER_TEST_SOURCES test/trajectory_optimizer_test.cpp)
    endif()
    ament_add_ros_isolated_gtest(test_diffusion_planner ${DIFFUSION_PLANNER_TEST_SOURCES})

    target_link_libraries(test_diffusion_planner autoware_diffusion_planner_component)
    ament_target_dependencies(test_diffusion_planner
      autoware_lanelet2_extension
      autoware_map_msgs
      autoware_perception_msgs
      autoware_test_utils
    )
  endif()
endif()
ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
  test_map
)
