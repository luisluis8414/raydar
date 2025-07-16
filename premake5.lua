-- premake5.lua

workspace "VoxVision"
    configurations { "Debug", "Release" }
    language "C++"
    cppdialect "C++17"
    
    -- Set the architecture
    architecture "x86_64"
    
    -- Set output directories
    targetdir "bin/%{cfg.buildcfg}"
    objdir "obj/%{cfg.buildcfg}"

    -- Clang specific configuration
    filter { "toolset:clang" }
        buildoptions {
            "-Wall",
            "-Wextra",
            "-std=c++17"
        }
        includedirs { 
            ".",
            "movement_detection/include",
            "deps",
            "deps/eigen-3.4.0",
            "/usr/include",
            "/usr/local/include"
        }

    -- Default configurations
    filter "configurations:Debug"
        defines { "DEBUG" }
        symbols "On"
        runtime "Debug"

    filter "configurations:Release"
        defines { "NDEBUG" }
        optimize "On"
        runtime "Release"

project "movement_detection"
    kind "StaticLib"
    location "movement_detection"
    
    files {
        "movement_detection/src/**.cpp",
        "movement_detection/include/**.hpp",
        "movement_detection/include/**.h"
    }
    
    includedirs {
        "%{wks.location}/movement_detection/include",
        "%{wks.location}/deps",
        "%{wks.location}/deps/eigen-3.4.0"
    }

    filter { "toolset:clang" }
        buildoptions { "-I%{wks.location}/movement_detection/include" }

project "Playground"
    kind "ConsoleApp"
    location "Playground"
    
    files { "%{prj.location}/**.cpp" }
    
    -- Link against movement_detection library
    links { "movement_detection" }
    includedirs { 
        "movement_detection/include",
        "deps",
        "deps/eigen-3.4.0"
    }
