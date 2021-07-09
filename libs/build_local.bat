@echo =================================================
@echo building mpir-windows
@echo =================================================
pushd %~dp0
pushd mpir-windows\msvc\vs19
cmd /C "msbuild.bat gc LIB x64 Release"
cmd /C "msbuild.bat cxx LIB x64 Release"
cmd /C "msbuild.bat gc LIB x64 Debug"
cmd /C "msbuild.bat cxx LIB x64 Debug"
popd
popd
