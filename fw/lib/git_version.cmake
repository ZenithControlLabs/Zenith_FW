# Run the Git command and store the result in GIT_VERSION
execute_process(
    COMMAND git describe --abbrev=7 --dirty --always --tags
    OUTPUT_STRIP_TRAILING_WHITESPACE
    OUTPUT_VARIABLE GIT_VERSION
)

set(GIT_VERSION_DEFINE "-DGIT_VERSION=\\\"${GIT_VERSION}\\\"")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${GIT_VERSION_DEFINE}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GIT_VERSION_DEFINE}")