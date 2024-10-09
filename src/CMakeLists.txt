# Add your source files here
add_library(incremental_bpa STATIC
    Facet.cpp
    Edge.cpp
    Vertex.cpp
    Point.cpp
    Mesher.cpp
    FileIO.cpp
)

# Add this line to ensure the header files are found. This means any executable that links to this library will also be able to find the header files.
target_include_directories(incremental_bpa PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})


#target_link_libraries(incremental_bpa PUBLIC ${Open3D_LIBRARIES})
