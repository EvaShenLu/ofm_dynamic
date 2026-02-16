add_rules("mode.release", "mode.debug")

includes("./../src/engine/xmake.lua")
includes("./../src/ofm/xmake.lua")

add_requires("vulkansdk", "glfw 3.4", "glm 1.0.1")
add_requires("glslang 1.3", { configs = { binaryonly = true } })
add_requires("imgui 1.91.1",  {configs = {glfw_vulkan = true}})
add_requires("cuda", {system=true, configs={utils={"cublas","cusparse","cusolver"}}})
-- On Windows use vcpkg VTK to avoid xmake VTK build and version mismatch (vcpkg has 9.3.0)
if is_plat("windows") then
    add_requires("vcpkg::vtk", {alias = "vtk"})
else
    add_requires("vtk 9.3.1")
end

set_policy("build.intermediate_directory", false)
set_runtimes("MD")
if is_plat("windows") then
    -- Required by FFmpeg/Boost and other Windows runtime dependencies.
    add_syslinks("Bcrypt", "Ole32", "Mfplat", "mfuuid", "Strmiids", "Secur32", "Crypt32", "Ncrypt", "User32", "ws2_32")
end

if os.isdir(path.join(os.scriptdir(), "sim_render")) then
    includes("sim_render")
end
includes("voxelization", "dynamic_obstacle")
add_options("compile_commands")

option("all")
    set_default(true)
    set_showmenu(false)
    set_description("Build all examples")