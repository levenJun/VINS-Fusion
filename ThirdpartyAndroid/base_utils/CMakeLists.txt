SET(LIB_NAME base_utils)
# PROJECT(${LIB_NAME})
CMAKE_MINIMUM_REQUIRED (VERSION 3.14.3)
SET(CMAKE_BUILD_TYPE RelWithDebInfo) # Release, RelWithDebInfo
# SET(CMAKE_VERBOSE_MAKEFILE OFF)

# Set sourcefiles
# LIST(APPEND SOURCEFILES
#         UtilsFile.cpp )
file(GLOB SOURCEFILES "*.cpp")


include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Create so library
ADD_LIBRARY(${LIB_NAME} SHARED ${SOURCEFILES})
TARGET_LINK_LIBRARIES(${LIB_NAME}  ${LINK_LIBS})    


################################################################################
# TESTS

# ADD_EXECUTABLE(${LIB_NAME}_TestAll test/TestAll.cpp)
# TARGET_LINK_LIBRARIES(${LIB_NAME}_TestAll ${LIB_NAME})