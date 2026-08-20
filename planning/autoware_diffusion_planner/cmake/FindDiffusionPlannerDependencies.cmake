include_guard(GLOBAL)

function(diffusion_planner_find_dependencies)
  find_package(CUDAToolkit REQUIRED)

  find_library(DIFFUSION_PLANNER_NVINFER_LIBRARY NAMES nvinfer)
  find_library(DIFFUSION_PLANNER_NVONNXPARSER_LIBRARY NAMES nvonnxparser)
  if(NOT DIFFUSION_PLANNER_NVINFER_LIBRARY OR
     NOT DIFFUSION_PLANNER_NVONNXPARSER_LIBRARY)
    message(FATAL_ERROR
      "TensorRT libraries (nvinfer and nvonnxparser) are required by ${PROJECT_NAME}")
  endif()

  add_library(DiffusionPlanner::nvinfer UNKNOWN IMPORTED)
  set_target_properties(DiffusionPlanner::nvinfer PROPERTIES
    IMPORTED_LOCATION "${DIFFUSION_PLANNER_NVINFER_LIBRARY}"
  )
  add_library(DiffusionPlanner::nvonnxparser UNKNOWN IMPORTED)
  set_target_properties(DiffusionPlanner::nvonnxparser PROPERTIES
    IMPORTED_LOCATION "${DIFFUSION_PLANNER_NVONNXPARSER_LIBRARY}"
  )

  if(DIFFUSION_PLANNER_VERBOSE_DEPENDENCIES)
    message(STATUS "CUDA toolkit: ${CUDAToolkit_VERSION}")
    message(STATUS "TensorRT inference library: ${DIFFUSION_PLANNER_NVINFER_LIBRARY}")
    message(STATUS "TensorRT ONNX parser: ${DIFFUSION_PLANNER_NVONNXPARSER_LIBRARY}")
  endif()

  set(has_onnxruntime FALSE)
  if(ENABLE_ONNXRUNTIME)
    find_package(onnxruntime QUIET)

    if(TARGET onnxruntime::onnxruntime)
      add_library(DiffusionPlanner::onnxruntime ALIAS onnxruntime::onnxruntime)
      set(has_onnxruntime TRUE)
    elseif(TARGET onnxruntime)
      add_library(DiffusionPlanner::onnxruntime ALIAS onnxruntime)
      set(has_onnxruntime TRUE)
    else()
      find_path(DIFFUSION_PLANNER_ONNXRUNTIME_INCLUDE_DIR
        NAMES onnxruntime_cxx_api.h
        HINTS /usr/local/include /usr/local/include/onnxruntime
      )
      find_library(DIFFUSION_PLANNER_ONNXRUNTIME_LIBRARY
        NAMES onnxruntime
        HINTS /usr/local/lib /usr/local/lib64
      )
      if(DIFFUSION_PLANNER_ONNXRUNTIME_INCLUDE_DIR AND
         DIFFUSION_PLANNER_ONNXRUNTIME_LIBRARY)
        add_library(DiffusionPlanner::onnxruntime UNKNOWN IMPORTED)
        set_target_properties(DiffusionPlanner::onnxruntime PROPERTIES
          IMPORTED_LOCATION "${DIFFUSION_PLANNER_ONNXRUNTIME_LIBRARY}"
          INTERFACE_INCLUDE_DIRECTORIES "${DIFFUSION_PLANNER_ONNXRUNTIME_INCLUDE_DIR}"
        )
        set(has_onnxruntime TRUE)
      endif()
    endif()

    if(has_onnxruntime)
      message(STATUS "ONNX Runtime is available; enabling ONNX Runtime inference support")
    else()
      message(WARNING
        "ONNX Runtime was not found; ONNX Runtime inference support will be disabled")
    endif()
  endif()

  set(DIFFUSION_PLANNER_HAS_ONNXRUNTIME ${has_onnxruntime} PARENT_SCOPE)
endfunction()
