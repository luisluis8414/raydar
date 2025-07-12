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
            "pixelToVoxel/include",
            "deps",
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

project "PixelToVoxel"
    kind "StaticLib"
    location "pixelToVoxel"
    
    files {
        "pixelToVoxel/src/**.cpp",
        "pixelToVoxel/include/**.hpp",
        "pixelToVoxel/include/**.h"
    }
    
    includedirs {
        "%{wks.location}/pixelToVoxel/include",
        "%{wks.location}/deps"
    }

    filter { "toolset:clang" }
        buildoptions { "-I%{wks.location}/pixelToVoxel/include" }

project "Playground"
    kind "ConsoleApp"
    location "Playground"
    
    files { "%{prj.location}/**.cpp" }
    
    -- Link against PixelToVoxel library
    links { "PixelToVoxel" }
    includedirs { 
        "pixelToVoxel/include",
        "deps"
    }
