file(GLOB_RECURSE sources "source/*.cpp" "source/*.c" "../../../common/casadi_interface.cpp")

add_library(casadigen_mhpc SHARED ${sources})

target_include_directories(casadigen_mhpc PUBLIC "header/")

if (DEBUG_MHPC)
    message("DEBUG_MODE is set for MHPC")
    target_compile_definitions(casadigen_mhpc PUBLIC DEBUG_MODE)
endif()


    
