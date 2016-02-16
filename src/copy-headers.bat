
del /S /Q ..\..\..\include\bullet\*.*

xcopy BulletCollision ..\..\..\include\bullet\BulletCollision\ /sy /exclude:copy-headers.exc
xcopy BulletDynamics ..\..\..\include\bullet\BulletDynamics\ /sy /exclude:copy-headers.exc
xcopy LinearMath ..\..\..\include\bullet\LinearMath\ /sy /exclude:copy-headers.exc
xcopy otbullet ..\..\..\include\bullet\otbullet\ /sy /exclude:copy-headers.exc

xcopy ..\bin\Debug ..\..\..\lib.12\Win32\Debug\ /sy /exclude:copy-headers.exc
xcopy ..\bin\Release ..\..\..\lib.12\Win32\Release\ /sy /exclude:copy-headers.exc
xcopy ..\bin\ReleaseLTCG ..\..\..\lib.12\Win32\ReleaseLTCG\ /sy /exclude:copy-headers.exc

xcopy ..\bin\ReleaseLTCG\otbullet.dll ..\..\..\..\bin\ /y
xcopy ..\bin\ReleaseLTCG\otbullet.pdb ..\..\..\..\bin\ /y