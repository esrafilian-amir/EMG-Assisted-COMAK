# The JAM tests are plain programs with their own main() (not Catch2 tests), so they
# are registered directly. (OpenSimAddTests, the macro used before, was removed upstream;
# OpenSimAddTest is for Catch2 tests.)
if(BUILD_TESTING)
    foreach(JAM_TEST testJAMComak testJAMForsim)
        add_executable(${JAM_TEST} ${JAM_TEST}.cpp)
        OpenSimConfigureTarget(${JAM_TEST})
        set_target_properties(${JAM_TEST} PROPERTIES FOLDER "Tests")
        target_link_libraries(${JAM_TEST} osimCommon osimSimulation
            osimAnalyses osimActuators osimLepton osimMoco osimJAM)
        add_test(NAME ${JAM_TEST} COMMAND ${JAM_TEST})
    endforeach()
endif()
