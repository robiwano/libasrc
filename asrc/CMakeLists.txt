
set(SOURCES
    src/asrc.cpp
)

set(HEADERS
    include/asrc/asrc.h
)

add_library(asrc ${ASRC_LINK_MODE} ${SOURCES} ${HEADERS})
target_link_libraries(asrc samplerate)

target_include_directories(asrc PUBLIC include)
