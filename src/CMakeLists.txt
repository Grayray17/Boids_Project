
#########################################################
# Source Files
#########################################################

# ----TODO------------------- #
# list your source files here #
# --------------------------- #
SET(sources
	"CMakeLists.txt"

	"application.hpp"
	"application.cpp"

	"boid.hpp"
	"boid.cpp"
	
	"scene.hpp"
	"scene.cpp"

	"main.cpp"
	"opengl.hpp"
)

# Add executable target and link libraries
add_executable(${CGRA_PROJECT} ${sources})

# ----TODO--------------------- #
# list your subdirectories here #
# ----------------------------- #
add_subdirectory(cgra)



#########################################################
# Link and Build Executable
#########################################################

# Set source groups (helper method)
target_source_group_tree(${CGRA_PROJECT})

# Set working directory
target_compile_definitions(${CGRA_PROJECT} PRIVATE "-DCGRA_WORKDIR=\"${PROJECT_SOURCE_DIR}/\"")

# Link usage requirements
target_link_libraries(${CGRA_PROJECT} PRIVATE glew glfw ${GLFW_LIBRARIES})
target_link_libraries(${CGRA_PROJECT} PRIVATE stb imgui)