workspace "ClosestPointQuery"
   startproject "Example"
   systemversion "latest"
   language "C++"
   cppdialect "C++11"
   toolset "msc" -- clang/gcc/msc
   configurations { "Debug", "Release" }
   platforms { "Win64" }
   buildoptions "/MT"
   
   objdir ("bin/obj")
   targetdir ("bin/%{cfg.platform}/%{cfg.buildcfg}")
   debugdir ("%{cfg.targetdir}")
   
   filter { "configurations:Debug" }
      defines { "DEBUG" }
      symbols "On"
   filter { "configurations:Release" }
      defines { "NDEBUG" }
      optimize "On"

   filter { "platforms:Win64" }
      architecture "x86_64"

project "ClosestPointQuery"
   kind "StaticLib"
   files { 
      "ClosestPointQuery/include/**.h", 
      "ClosestPointQuery/src/**.cpp",
   }
   includedirs { 
      "ClosestPointQuery/include",
      "ClosestPointQuery/lib/glm",
      "ClosestPointQuery/lib/RTree"
   }

project "Example"
   kind "ConsoleApp"
   files { 
      "ClosestPointQuery/include/**.h", 
      "ClosestPointQuery/src/**.cpp",
      "ClosestPointQuery/example/**.cpp"
   }
   includedirs { 
      "ClosestPointQuery/include",
      "ClosestPointQuery/lib/glm",
      "ClosestPointQuery/lib/tinyobjloader",
      "ClosestPointQuery/lib/RTree",
   }
   
project "UnitTest"
   kind "ConsoleApp"
   links {
      "gtest",
      "ClosestPointQuery"
   }
   libdirs {
      "%{cfg.targetdir}",
      "ClosestPointQuery/lib/googletest/lib"
   }
   files { 
      "ClosestPointQuery/test/**.cpp"
   }
   includedirs { 
      "ClosestPointQuery/include",
      "ClosestPointQuery/lib/glm",
      "ClosestPointQuery/lib/tinyobjloader",
      "ClosestPointQuery/lib/RTree",
      "ClosestPointQuery/lib/googletest/include"
   }