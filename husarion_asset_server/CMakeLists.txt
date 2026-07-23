cmake_minimum_required(VERSION 3.19) # file(CHMOD ...)
project(husarion_asset_server)

find_package(ament_cmake REQUIRED)

# Bump in lockstep with package.xml <version> when husarion_asset_server cuts a
# new release: https://github.com/husarion/husarion_asset_server/releases
set(ASSET_SERVER_VERSION "0.3.0")

execute_process(
  COMMAND dpkg --print-architecture
  OUTPUT_VARIABLE ASSET_SERVER_ARCH
  OUTPUT_STRIP_TRAILING_WHITESPACE)

set(_asset_server_base
    "https://github.com/husarion/husarion_asset_server/releases/download/v${ASSET_SERVER_VERSION}"
)
set(_asset_server_file
    "asset_server-${ASSET_SERVER_VERSION}-linux-${ASSET_SERVER_ARCH}")
set(_asset_server_bin "${CMAKE_CURRENT_BINARY_DIR}/asset_server")

if(NOT EXISTS "${_asset_server_bin}")
  file(DOWNLOAD "${_asset_server_base}/${_asset_server_file}.sha256"
       "${_asset_server_bin}.sha256")
  file(STRINGS "${_asset_server_bin}.sha256" _asset_server_hash_line
       LIMIT_COUNT 1)
  string(REGEX MATCH "^[0-9a-f]+" _asset_server_hash
               "${_asset_server_hash_line}")
  if(NOT _asset_server_hash)
    message(
      FATAL_ERROR "Could not parse sha256 from ${_asset_server_file}.sha256")
  endif()

  file(
    DOWNLOAD "${_asset_server_base}/${_asset_server_file}"
    "${_asset_server_bin}"
    EXPECTED_HASH "SHA256=${_asset_server_hash}"
    STATUS _asset_server_dl_status)
  list(GET _asset_server_dl_status 0 _asset_server_dl_code)
  if(NOT _asset_server_dl_code EQUAL 0)
    list(GET _asset_server_dl_status 1 _asset_server_dl_message)
    file(REMOVE "${_asset_server_bin}")
    message(
      FATAL_ERROR
        "Failed to download ${_asset_server_file}: ${_asset_server_dl_message}")
  endif()

  # install(PROGRAMS ...) only chmods +x on a real copy; --symlink-install (this
  # repo's default colcon invocation) symlinks straight to this file instead, so
  # the build-tree copy itself must already be executable.
  file(
    CHMOD
    "${_asset_server_bin}"
    PERMISSIONS
    OWNER_READ
    OWNER_WRITE
    OWNER_EXECUTE
    GROUP_READ
    GROUP_EXECUTE
    WORLD_READ
    WORLD_EXECUTE)
endif()

install(PROGRAMS "${_asset_server_bin}" DESTINATION lib/${PROJECT_NAME})

ament_package()
