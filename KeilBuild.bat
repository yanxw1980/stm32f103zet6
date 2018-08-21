@echo off
set UV=C:\Keil_v5\UV4\UV4.exe
set UV_PRO_PATH=%cd%\Keil5\LoaderPrj.uvprojx
echo Init building ...
echo .>build_log.txt
%UV% -j0 -r %UV_PRO_PATH% -o %cd%\build_log.txt
type build_log.txt
echo Done.
pause
