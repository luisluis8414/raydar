-- premake5.lua

workspace "Raydar"
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
            "raydar/include",
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

project "raydar"
    kind "StaticLib"
    location "raydar"
    
    files {
        "raydar/src/**.cpp",
        "raydar/include/**.hpp",
        "raydar/include/**.h"
    }
    
    includedirs {
        "%{wks.location}/raydar/include",
        "%{wks.location}/deps",
        "%{wks.location}/deps/eigen-3.4.0"
    }

    filter { "toolset:clang" }
        buildoptions { "-I%{wks.location}/raydar/include" }

project "Playground"
    kind "ConsoleApp"
    location "Playground"
    
    files { "%{prj.location}/**.cpp" }
    
    -- Link against raydar library
    links { "raydar" }
    includedirs { 
        "raydar/include",
        "deps",
        "deps/eigen-3.4.0"
    }
