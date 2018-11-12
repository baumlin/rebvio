FIND_PATH(
  spdlog_INCLUDE_DIRS
  NAMES spdlog/spdlog.h
  PATHS ${PROJECT_SOURCE_DIR}/../spdlog/include
)
IF(spdlog_INCLUDE_DIRS)
  SET(spdlog_FOUND TRUE)
  MESSAGE("[${PROJECT_NAME}]: Found spdlog: ${spdlog_INCLUDE_DIRS}")
ELSE()
  SET(spdlog_FOUND FALSE)
  MESSAGE(FATAL_ERROR "[${PROJECT_NAME}]: Could not find spdlog in ${PROJECT_SOURCE_DIR}/../spdlog/include")
ENDIF()
