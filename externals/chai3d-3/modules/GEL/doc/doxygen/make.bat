@echo off
setlocal enableextensions enabledelayedexpansion

REM Software License Agreement (BSD License)
REM Copyright (c) 2003-2023, CHAI3D
REM (www.chai3d.org)
REM
REM All rights reserved.
REM
REM Redistribution and use in source and binary forms, with or without
REM modification, are permitted provided that the following conditions
REM
REM * Redistributions of source code must retain the above copyright
REM notice, this list of conditions and the following disclaimer.
REM
REM * Redistributions in binary form must reproduce the above
REM copyright notice, this list of conditions and the following
REM disclaimer in the documentation and/or other materials provided
REM with the distribution.
REM
REM * Neither the name of CHAI3D nor the names of its contributors may
REM be used to endorse or promote products derived from this software
REM without specific prior written permission.
REM
REM THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
REM "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
REM LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
REM FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
REM COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
REM INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
REM BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
REM LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
REM CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
REM LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
REM ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
REM POSSIBILITY OF SUCH DAMAGE.

REM define top level folder
set top=..\..

REM generate versioned files for all *.ver files
call %top%\toolchains\scripts\version.bat %top%\project
if !ERRORLEVEL! neq 0 goto error

REM generate HTML doc for all doxygen source files
for %%f in (*doxyfile) do (
    set file=%%f
    set line='type !file! ^| findstr "HTML_OUTPUT"'
    for /f "tokens=1,2,3" %%a in (!line!) do (
        if not "%%a" == "#" (set html_output=%%c)
    )
    call doxygen !file!
    if !ERRORLEVEL! neq 0 goto error
    call %top%\toolchains\scripts\version.bat %top%\project %html_output%
    if !ERRORLEVEL! neq 0 goto error
)
goto eof

:error
echo *** error - see log file for more information
pause
goto eof

:eof
endlocal
