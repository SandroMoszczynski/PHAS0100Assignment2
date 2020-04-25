#/*============================================================================
#
#  PHAS0100ASSIGNMENT2: PHAS0100 Assignment 2 Social Force Model
#
#  Copyright (c) University College London (UCL). All rights reserved.
#
#  This software is distributed WITHOUT ANY WARRANTY; without even
#  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
#  PURPOSE.
#
#  See LICENSE.txt in the top level directory for details.
#
#============================================================================*/


set(_command_line_apps
  sfmMyFirstApp
  sfmPedestrianSpawner
)
add_executable(sfmOpen_MP.x sfmOpen_MP.cpp)

find_package(OpenMP REQUIRED)

if(OPENMP_FOUND)
  target_compile_options(sfmOpen_MP.x PUBLIC "${OpenMP_CXX_FLAGS}")
  target_link_libraries(sfmOpen_MP.x "${OpenMP_CXX_FLAGS}")
  target_link_libraries(sfmOpen_MP.x ${PHAS0100ASSIGNMENT2_LIBRARIES} ${ALL_THIRD_PARTY_LIBRARIES})
endif()

if (VTK_FOUND)
  list(APPEND _command_line_apps sfmVisualiser)
  list(APPEND _command_line_apps sfmVisualiserDemo)
endif ()

foreach(_app ${_command_line_apps})
  add_executable(${_app} ${_app}.cpp)
  target_link_libraries(${_app} ${PHAS0100ASSIGNMENT2_LIBRARIES} ${ALL_THIRD_PARTY_LIBRARIES})
endforeach()
