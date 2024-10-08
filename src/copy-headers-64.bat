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
