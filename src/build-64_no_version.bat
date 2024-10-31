
@echo off
setlocal enableDelayedExpansion

echo Setting up the environment
call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvarsall.bat" x86_amd64


msbuild otbullet\otbullet.sln /m /target:Build /p:Configuration=Debug /p:Platform=x64

set BUILD_STATUS=%ERRORLEVEL%
echo build status (x64 Debug) %BUILD_STATUS%
if "%BUILD_STATUS%" neq "0" goto :fail


msbuild otbullet\otbullet.sln /m /target:Build /p:Configuration=ReleaseLTCG /p:Platform=x64

set BUILD_STATUS=%ERRORLEVEL%
echo build status (x64 ReleaseLTCG) %BUILD_STATUS%
if "%BUILD_STATUS%" neq "0" goto :fail


del /S /Q /F ..\..\..\include\bullet\*.*

xcopy BulletCollision ..\..\..\include\bullet\BulletCollision\ /sy /exclude:copy-headers.exc
xcopy BulletDynamics ..\..\..\include\bullet\BulletDynamics\ /sy /exclude:copy-headers.exc
xcopy LinearMath ..\..\..\include\bullet\LinearMath\ /sy /exclude:copy-headers.exc
xcopy *.h ..\..\..\include\bullet\ /y
xcopy otbullet\physics.h ..\..\..\include\bullet\otbullet\ /y
xcopy otbullet\physics_cfg.h ..\..\..\include\bullet\otbullet\ /y
xcopy otbullet\otflags.h ..\..\..\include\bullet\otbullet\ /y
xcopy otbullet\shape_info_cfg.h ..\..\..\include\bullet\otbullet\ /y
xcopy otbullet\docs\*.html ..\..\..\include\bullet\otbullet\docs\ /sy

xcopy ..\bin\x64\ReleaseLTCG\otbullet.dll ..\..\..\..\bin\ /y
xcopy ..\bin\x64\ReleaseLTCG\otbullet.pdb ..\..\..\..\bin\ /y
xcopy ..\bin\x64\Debug\otbulletd.dll ..\..\..\..\bin\ /y
xcopy ..\bin\x64\Debug\otbulletd.pdb ..\..\..\..\bin\ /y

goto :eof

:fail
echo Build failed or cancelled
pause > nul
goto :eof
