workspace "ClosestPointQuery"
   configurations { "Debug", "Release" }

project "ClosestPointQuery"
   kind "ConsoleApp"
   language "C++"
   objdir ("bin/obj")
   files { "ClosestPointQuery/src/**.h", "ClosestPointQuery/src/**.cpp" }
   includedirs { 
      "ClosestPointQuery/lib/tinyobjloader",
      "ClosestPointQuery/lib/RTree",
      "ClosestPointQuery/lib/glm"
   }

   filter { "configurations:Debug" }
      defines { "DEBUG" }
      symbols "On"

   filter { "configurations:Release" }
      defines { "NDEBUG" }
      optimize "On"