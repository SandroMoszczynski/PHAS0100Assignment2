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

set(DATA_DIR ${CMAKE_SOURCE_DIR}/Testing/Data)
set(TMP_DIR ${CMAKE_BINARY_DIR}/Testing/Temporary)

set(TEST_CASES
  sfmBasicTest
  sfmCommandLineArgsTest
  sfmTypesTest
  sfmPHASTest
)

foreach(_test_case ${TEST_CASES})
  add_executable(${_test_case} ${_test_case}.cpp sfmCatchMain.cpp)
  target_link_libraries(${_test_case} ${PHAS0100ASSIGNMENT2_LIBRARIES} ${ALL_THIRD_PARTY_LIBRARIES})
endforeach()

# Adding tests is done outside the above loop,
# as each test will need different arguments.

##################################################################################################
# Dont forget its:  add_test(<test name (globally unique) > <exe name> <argument1> <argument2>
##################################################################################################

add_test(NoArgs ${EXECUTABLE_OUTPUT_PATH}/sfmBasicTest)
add_test(NoArgs ${EXECUTABLE_OUTPUT_PATH}/sfmPHASTest)
add_test(NoArgs ${EXECUTABLE_OUTPUT_PATH}/sfmTypesTest)
add_test(1File ${EXECUTABLE_OUTPUT_PATH}/sfmCommandLineArgsTest ${DATA_DIR}/input.txt)
