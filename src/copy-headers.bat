
del /S /Q ..\..\..\include\bullet\*.*

xcopy BulletCollision ..\..\..\include\bullet\BulletCollision\ /sy /exclude:copy-headers.exc
xcopy BulletDynamics ..\..\..\include\bullet\BulletDynamics\ /sy /exclude:copy-headers.exc
xcopy LinearMath ..\..\..\include\bullet\LinearMath\ /sy /exclude:copy-headers.exc
xcopy *.h ..\..\..\include\bullet\ /y
xcopy otbullet ..\..\..\include\bullet\otbullet\ /sy /exclude:copy-headers.exc

rem xcopy ..\bin\Win32\Debug ..\..\..\lib.14\Win32\Debug\ /sy /exclude:copy-headers.exc
rem xcopy ..\bin\Win32\Release ..\..\..\lib.14\Win32\Release\ /sy /exclude:copy-headers.exc
rem xcopy ..\bin\Win32\ReleaseLTCG ..\..\..\lib.14\Win32\ReleaseLTCG\ /sy /exclude:copy-headers.exc

xcopy ..\bin\Win32\ReleaseLTCG\otbullet.dll ..\..\..\..\bin\ /y
xcopy ..\bin\Win32\ReleaseLTCG\otbullet.pdb ..\..\..\..\bin\ /y
xcopy ..\bin\Win32\Debug\otbulletd.dll ..\..\..\..\bin\ /y
xcopy ..\bin\Win32\Debug\otbulletd.pdb ..\..\..\..\bin\ /y

xcopy ..\bin\x64\ReleaseLTCG\otbullet.dll ..\..\..\..\bin64\ /y
xcopy ..\bin\x64\ReleaseLTCG\otbullet.pdb ..\..\..\..\bin64\ /y
xcopy ..\bin\x64\Debug\otbulletd.dll ..\..\..\..\bin64\ /y
xcopy ..\bin\x64\Debug\otbulletd.pdb ..\..\..\..\bin64\ /y
