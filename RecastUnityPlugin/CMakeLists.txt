file(GLOB SOURCES Source/*.cpp Contrib/fastlz/fastlz.c)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

include_directories(SYSTEM Contrib/fastlz)
include_directories(SYSTEM Contrib)
include_directories(../DebugUtils/Include)
include_directories(../Detour/Include)
include_directories(../DetourCrowd/Include)
include_directories(../DetourTileCache/Include)
include_directories(../Recast/Include)
include_directories(Include)

if (WIN32)
    add_library(RecastUnityPlugin SHARED WIN32 ${SOURCES})
elseif(APPLE)
    add_library(RecastUnityPlugin SHARED MACOSX_BUNDLE ${SOURCES})
else()
    add_library(RecastUnityPlugin SHARED  ${SOURCES})
endif()

if( WIN32 )
  if ( "${CMAKE_MAKE_PROGRAM}" MATCHES "MSBuild" )
    add_custom_command(TARGET RecastUnityPlugin
      POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/Bin/Meshes ${CMAKE_BINARY_DIR}/RecastUnityPlugin/$(ConfigurationName)/Meshes
      COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/Bin/TestCases ${CMAKE_BINARY_DIR}/RecastUnityPlugin/$(ConfigurationName)/TestCases
      COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/Bin/DroidSans.ttf ${CMAKE_BINARY_DIR}/RecastUnityPlugin/$(ConfigurationName)/
    )
  elseif ( MINGW )
    add_custom_command(TARGET RecastUnityPlugin
      POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy "${SDL2_RUNTIME_LIBRARY}" ${CMAKE_BINARY_DIR}/RecastUnityPlugin/
      COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/Bin/Meshes ${CMAKE_BINARY_DIR}/RecastUnityPlugin/Meshes
      COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/Bin/TestCases ${CMAKE_BINARY_DIR}/RecastUnityPlugin/TestCases
      COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/Bin/DroidSans.ttf ${CMAKE_BINARY_DIR}/RecastUnityPlugin/
    )
  endif()
else()
    file(COPY Bin/Meshes DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
    file(COPY Bin/TestCases DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
    file(COPY Bin/DroidSans.ttf DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
endif()


add_dependencies(RecastUnityPlugin DebugUtils Detour DetourCrowd DetourTileCache Recast)
if(APPLE)
  target_link_libraries(RecastUnityPlugin DebugUtils Detour DetourCrowd DetourTileCache Recast)
else()
  target_link_libraries(RecastUnityPlugin DebugUtils Detour DetourCrowd DetourTileCache Recast)
endif()


install(TARGETS RecastUnityPlugin
        RUNTIME DESTINATION bin
        BUNDLE DESTINATION bin)
install(DIRECTORY Bin/Meshes DESTINATION bin)
install(DIRECTORY Bin/TestCases DESTINATION bin)
install(FILES Bin/DroidSans.ttf DESTINATION bin)