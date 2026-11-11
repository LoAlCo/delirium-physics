@echo off
rem Builds rdr2nmx.asi (x64, release) with the VS 2022 Build Tools.
rem Needs the Script Hook RDR 2 SDK unpacked into sdk\ next to this file (sdk\inc, sdk\lib); see README.md.
call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\Common7\Tools\VsDevCmd.bat" -arch=amd64 -no_logo >nul
cd /d "%~dp0"
if not exist sdk\lib\ScriptHookRDR2.lib (echo Script Hook RDR 2 SDK not found in sdk\: see README.md & exit /b 1)
if not exist out mkdir out
cl /nologo /std:c++17 /O2 /EHa /MT /W3 /LD src\rdr2nmx.cpp sdk\lib\ScriptHookRDR2.lib /Fo:out\ /Fe:out\rdr2nmx.asi /link /NOLOGO
