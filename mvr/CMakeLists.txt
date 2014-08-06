set(Boost_USE_STATIC_LIBS  ON)
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF)

find_package(3rdParty)
find_package(Boost REQUIRED system thread chrono)
find_package(CGAL REQUIRED)
find_package(OpenSceneGraph REQUIRED osgViewer osgText osgDB osgGA osgQt osgManipulator osgUtil)
find_package(Qt4 REQUIRED QtCore QtGui QtOpenGL QtXml)
find_package(PCL REQUIRED common io registration kdtree search)


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

link_directories(${CGAL_LIBRARIES_DIR})

include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

if(OPENSCENEGRAPH_FOUND)
    include_directories(${OPENSCENEGRAPH_INCLUDE_DIRS})
endif(OPENSCENEGRAPH_FOUND)

if(CGAL_FOUND)
  include_directories(${CGAL_INCLUDE_DIRS})
  set(CMAKE_MODULE_PATH ${CGAL_DIR}/cmake/modules)
  set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CGAL_DIR})
  include(CGAL_Macros)
  find_package(GMP REQUIRED)
  if(GMP_FOUND)
    include_directories(${GMP_INCLUDE_DIR})
  endif(GMP_FOUND)
endif(CGAL_FOUND)

if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
endif(Boost_FOUND)

include(${QT_USE_FILE})

# Set Qt files and resources here
set (ui         resource/main_window.ui)
set (moc_incs   include/main_window.h
				include/point_cloud.h
				include/file_system_model.h
				include/threaded_painter.h
				include/osg_viewer_widget.h
				include/registrator.h
				include/parameter_dialog.h)
set (resources  resource/main_window.qrc)

# Qt stuff
QT4_WRAP_UI(ui_srcs ${ui})
QT4_WRAP_CPP(moc_srcs ${moc_incs} OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED)
QT4_ADD_RESOURCES(resource_srcs ${resources})

set (incs       ${moc_incs}
				include/types.h
				include/file_viewer_widget.h
				include/light_source.h
				include/renderable.h
				include/update_callback.h
				include/adapter_widget.h
				include/toggle_handler.h
				include/parameter_manager.h
				include/parameter.h	
				include/update_visitor.h
				include/osg_utility.h
				include/math_solvers.h
				include/impl/parameter.hpp
                )

set (srcs 		${ui_srcs}
				${moc_srcs}
				${resource_srcs}
                src/main.cpp
				src/main_window.cpp
				src/point_cloud.cpp
				src/file_system_model.cpp
				src/file_viewer_widget.cpp
				src/threaded_painter.cpp
				src/osg_viewer_widget.cpp
				src/light_source.cpp
				src/renderable.cpp
				src/update_callback.cpp
				src/adapter_widget.cpp
				src/registrator.cpp
				src/toggle_handler.cpp
				src/parameter_manager.cpp
				src/parameter.cpp
				src/parameter_dialog.cpp
				src/update_visitor.cpp
				src/osg_utility.cpp
				src/math_solvers.cpp
				)

# Organize files
SOURCE_GROUP("Resources" FILES ${ui} ${resources})
SOURCE_GROUP("Generated" FILES ${ui_srcs} ${moc_srcs} ${resource_srcs})
#SET_SOURCE_FILES_PROPERTIES(${srcs} PROPERTIES OBJECT_DEPENDS "${ui_srcs}")

# Put the ui in the windows project file
IF (${CMAKE_BUILD_TOOL} MATCHES "msdev")
  SET (srcs ${srcs} ${uis})
ENDIF (${CMAKE_BUILD_TOOL} MATCHES "msdev")
IF (${CMAKE_BUILD_TOOL} MATCHES "devenv")
  SET (srcs ${srcs} ${uis})
ENDIF (${CMAKE_BUILD_TOOL} MATCHES "devenv")

set(exe_name mvr)
add_executable(${exe_name} ${srcs} ${incs})
target_link_libraries(${exe_name} ${Boost_THREAD_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${Boost_CHRONO_LIBRARY} ${OPENSCENEGRAPH_LIBRARIES} ${QT_QTOPENGL_LIBRARY} ${QT_QTGUI_LIBRARY} ${QT_QTXML_LIBRARY} ${QT_QTCORE_LIBRARY} ${CGAL_LIBRARIES}
${GMP_LIBRARIES} ${PCL_COMMON_LIBRARY} ${PCL_IO_LIBRARY} ${PCL_REGISTRATION_LIBRARY} ${PCL_KDTREE_LIBRARY} ${PCL_SEARCH_LIBRARY} ${ThirdParty_LIBS})

if(WIN32 AND MSVC)
  set_target_properties(${exe_name} PROPERTIES LINK_FLAGS_RELEASE /OPT:REF)
elseif(CMAKE_SYSTEMname STREQUAL "Darwin")
  if(NOT CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    set_target_properties(${exe_name} PROPERTIES LINK_FLAGS -Wl)
  endif()
elseif(__COMPILER_PATHSCALE)
  set_target_properties(${exe_name} PROPERTIES LINK_FLAGS -mp)
else()
  set_target_properties(${exe_name} PROPERTIES LINK_FLAGS -Wl,--as-needed)
endif()

set_target_properties(${exe_name} PROPERTIES DEBUG_POSTFIX _debug)
set_target_properties(${exe_name} PROPERTIES RELEASE_POSTFIX _release)
