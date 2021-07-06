@echo =================================================
@echo building glog
@echo =================================================
pushd %~dp0
mkdir glog_vsbuild
pushd glog_vsbuild
cmake -DBUILD_TESTING:BOOL=OFF -DWITH_GFLAGS:BOOL=OFF ..\glog -G "Visual Studio 16 2019" || exit 1
MSBuild glog.sln -target:glog -p:Configuration=Release;Platform=x64 || exit 1
MSBuild glog.sln -target:glog -p:Configuration=Debug;Platform=x64 || exit 1
popd
popd
