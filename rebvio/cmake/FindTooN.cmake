FIND_PATH(
  TooN_INCLUDE_DIRS
  NAMES TooN/TooN.h
  PATHS ${PROJECT_SOURCE_DIR}/../TooN
)
IF(TooN_INCLUDE_DIRS)
  SET(TooN_FOUND TRUE)
  MESSAGE("[${PROJECT_NAME}]: Found TooN: ${TooN_INCLUDE_DIRS}")
ELSE()
  SET(Toon_FOUND FALSE)
  MESSAGE(FATAL_ERROR "[${PROJECT_NAME}]: Could not find TooN in ${PROJECT_SOURCE_DIR}/../TooN")
ENDIF()
