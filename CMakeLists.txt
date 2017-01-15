cmake_minimum_required(VERSION 3.5)

set( PROJECTNAME mcplane )

SET( CMAKE_CXX_FLAGS "-std=c++11")

include_directories( . )
link_directories( . )

file( GLOB source_files *.cpp *.hpp *.inl )

add_executable( ${PROJECTNAME} ${source_files} )

target_link_libraries( ${PROJECTNAME}
	SDL2
	SDL2_image
	SDL2main
	GL
	GLU
	GLEW

	#PhysXLoader
	#PhysX3_64
	#PhysX3Common_64
	PhysX3DEBUG_x64
	PhysX3CommonDEBUG_x64
	PhysX3CookingDEBUG_x64
	PhysX3CharacterKinematicDEBUG_x64
	PhysX3ExtensionsDEBUG
	PhysX3VehicleDEBUG
	PhysXProfileSDKDEBUG
	PhysXVisualDebuggerSDKDEBUG
	PxTaskDEBUG
	)

add_definitions(
	-D_DEBUG
	-DPX_DEBUG
	-DPX_CHECKED
	-DPX_SUPPORT_VISUAL_DEBUGGER
	)

