project(TCPClient_App LANGUAGES C CXX)
include(FetchContent)

# Portable EXE on MinGW
if(MINGW)
  add_link_options(-static -static-libgcc -static-libstdc++)
endif()
set(BUILD_SHARED_LIBS OFF)

# ---- Paths relative to this subdir ----
set(APP_DIR   "${CMAKE_CURRENT_SOURCE_DIR}")
set(SRC_DIR   "${APP_DIR}/src")
set(IMGUI_DIR "${APP_DIR}/Dependencies/imgui")
set(GLAD_DIR  "${APP_DIR}/Dependencies/glad")
set(STB_DIR   "${APP_DIR}/Dependencies/stb_image")

file(GLOB TCPCLIENT_SRC CONFIGURE_DEPENDS
  "${SRC_DIR}/*.cpp"
  "${SRC_DIR}/UI/*.cpp"
  "${SRC_DIR}/UI/Panels/*.cpp"
  "${SRC_DIR}/video/*.cpp"
)
list(APPEND TCPCLIENT_SRC
  "${GLAD_DIR}/src/glad.c"
  "${IMGUI_DIR}/imgui.cpp"
  "${IMGUI_DIR}/imgui_draw.cpp"
  "${IMGUI_DIR}/imgui_tables.cpp"
  "${IMGUI_DIR}/imgui_widgets.cpp"
  "${IMGUI_DIR}/backends/imgui_impl_glfw.cpp"
  "${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp"
  "${STB_DIR}/stb_image.cpp"
)

add_executable(TCPClient ${TCPCLIENT_SRC})
target_compile_features(TCPClient PRIVATE cxx_std_17)
target_include_directories(TCPClient PRIVATE
  "${APP_DIR}"
  "${SRC_DIR}"
  "${SRC_DIR}/UI"
  "${SRC_DIR}/UI/Panels"
  "${SRC_DIR}/video"
  "${IMGUI_DIR}"
  "${IMGUI_DIR}/backends"
  "${GLAD_DIR}/include"
  "${STB_DIR}"
)
target_compile_definitions(TCPClient PRIVATE
  IMGUI_IMPL_OPENGL_LOADER_CUSTOM
  GLFW_INCLUDE_NONE
  GLFW_STATIC
  $<$<CONFIG:Debug>:DEBUG>
)

# ---- OpenGL ----
find_package(OpenGL REQUIRED)
target_link_libraries(TCPClient PRIVATE OpenGL::GL)

# ---- GLFW: local if present, else fetch ----
set(GLFW_LOCAL_INCLUDE "${APP_DIR}/Dependencies/glfw/include")
set(GLFW_LOCAL_LIB     "${APP_DIR}/Dependencies/glfw/lib-mingw/libglfw3.a")
set(GLFW_LOCAL_SOURCE  "${APP_DIR}/Dependencies/glfw")
if(NOT DEPS_FORCE_FETCH AND EXISTS "${GLFW_LOCAL_INCLUDE}" AND EXISTS "${GLFW_LOCAL_LIB}")
  add_library(glfw_local STATIC IMPORTED GLOBAL)
  set_target_properties(glfw_local PROPERTIES
    IMPORTED_LOCATION "${GLFW_LOCAL_LIB}"
    INTERFACE_INCLUDE_DIRECTORIES "${GLFW_LOCAL_INCLUDE}"
  )
  add_library(glfw ALIAS glfw_local)
else()
  set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_TESTS    OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_DOCS     OFF CACHE BOOL "" FORCE)
  set(GLFW_LIBRARY_TYPE STATIC CACHE STRING "" FORCE)
  if(NOT DEPS_FORCE_FETCH AND EXISTS "${GLFW_LOCAL_SOURCE}/CMakeLists.txt")
    add_subdirectory("${GLFW_LOCAL_SOURCE}" "${CMAKE_CURRENT_BINARY_DIR}/glfw-build")
  else()
    FetchContent_Declare(glfw_src
      GIT_REPOSITORY https://github.com/glfw/glfw.git
      GIT_TAG 3.4
    )
    FetchContent_MakeAvailable(glfw_src) # provides 'glfw'
  endif()
endif()

# ---- Link third-party + shared + system libs ----
target_link_libraries(TCPClient PRIVATE shared)
target_link_libraries(TCPClient PRIVATE glfw shared)
target_link_libraries(TCPClient PRIVATE
  ws2_32 crypt32 bcrypt gdi32 dinput8 dxguid
)
if(MINGW)
  target_link_libraries(TCPClient PRIVATE winmm uxtheme)
endif()

# --- RTSP toggle ---
option(ENABLE_RTSP "Enable RTSP via FFmpeg" OFF)

# Define macro for this target: 1 if ON, 0 if OFF
target_compile_definitions(TCPClient PRIVATE ENABLE_RTSP=$<BOOL:${ENABLE_RTSP}>)

if (ENABLE_RTSP)
  if (NOT DEFINED FFMPEG_ROOT)
    message(FATAL_ERROR "ENABLE_RTSP=ON but FFMPEG_ROOT is not set")
  endif()

  # Headers
  target_include_directories(TCPClient PRIVATE
    "${FFMPEG_ROOT}/include"
  )

  # Link libs
  target_link_libraries(TCPClient PRIVATE
    "${FFMPEG_ROOT}/lib/libavformat.dll.a"
    "${FFMPEG_ROOT}/lib/libavcodec.dll.a"
    "${FFMPEG_ROOT}/lib/libavutil.dll.a"
    "${FFMPEG_ROOT}/lib/libswscale.dll.a"
  )

  # Copy DLLs
  if (FFMPEG_COPY_DLLS)
    file(GLOB FFMPEG_DLLS
      "${FFMPEG_ROOT}/bin/avformat-*.dll"
      "${FFMPEG_ROOT}/bin/avcodec-*.dll"
      "${FFMPEG_ROOT}/bin/avutil-*.dll"
      "${FFMPEG_ROOT}/bin/swscale-*.dll"
      "${FFMPEG_ROOT}/bin/swresample-*.dll"
    )

    add_custom_command(TARGET TCPClient POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_if_different
              ${FFMPEG_DLLS} "$<TARGET_FILE_DIR:TCPClient>"
      COMMENT "Copying minimal FFmpeg DLLs"
      COMMAND_EXPAND_LISTS
    )
  endif()
endif()

# Output + assets
set_target_properties(TCPClient PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${APP_DIR}/build"
  OUTPUT_NAME "TCPClient"
)
add_custom_command(TARGET TCPClient POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_directory
          "${APP_DIR}/assets"
          "$<TARGET_FILE_DIR:TCPClient>/assets"
)
