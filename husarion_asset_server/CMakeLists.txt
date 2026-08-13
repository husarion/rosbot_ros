cmake_minimum_required(VERSION 3.19) # file(CHMOD ...)
project(husarion_asset_server)

find_package(ament_cmake REQUIRED)

# Sole pin for the fetched binary — independent of package.xml <version>, which
# tracks rosbot_ros releases and is rewritten repo-wide by the release tooling.
# Releases: https://github.com/husarion/husarion_asset_server/releases
set(ASSET_SERVER_VERSION "0.4.0")

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

# GitHub's release-asset CDN intermittently 503s (seen in rosbot-snap CI).
# file(DOWNLOAD) has no built-in retry, so wrap it here and always check STATUS
# — without that, an HTTP error still writes its body to the destination file
# and the *next* command (parsing/hashing) fails with a misleading error instead
# of the actual HTTP status.
function(_asset_server_download_with_retry url dest)
  set(_extra_args ${ARGN})
  set(_attempts 3)
  foreach(_attempt RANGE 1 ${_attempts})
    file(
      DOWNLOAD "${url}" "${dest}"
      STATUS _dl_status
      ${_extra_args})
    list(GET _dl_status 0 _dl_code)
    if(_dl_code EQUAL 0)
      return()
    endif()
    file(REMOVE "${dest}")
    if(_attempt LESS ${_attempts})
      execute_process(COMMAND "${CMAKE_COMMAND}" -E sleep 5)
    endif()
  endforeach()
  list(GET _dl_status 1 _dl_message)
  message(
    FATAL_ERROR
      "Failed to download ${url} after ${_attempts} attempts: ${_dl_message}")
endfunction()

if(NOT EXISTS "${_asset_server_bin}")
  _asset_server_download_with_retry(
    "${_asset_server_base}/${_asset_server_file}.sha256"
    "${_asset_server_bin}.sha256")
  file(STRINGS "${_asset_server_bin}.sha256" _asset_server_hash_line
       LIMIT_COUNT 1)
  string(REGEX MATCH "^[0-9a-f]+" _asset_server_hash
               "${_asset_server_hash_line}")
  if(NOT _asset_server_hash)
    message(
      FATAL_ERROR "Could not parse sha256 from ${_asset_server_file}.sha256")
  endif()

  _asset_server_download_with_retry(
    "${_asset_server_base}/${_asset_server_file}" "${_asset_server_bin}"
    EXPECTED_HASH "SHA256=${_asset_server_hash}")

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
