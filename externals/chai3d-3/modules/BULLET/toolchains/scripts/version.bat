@echo off
setlocal enableextensions enabledelayedexpansion

REM  Copyright (C) 2001-2022 Force Dimension, Switzerland.
REM  All Rights Reserved.
REM
REM  $PROJECT $VERSION
REM  $DESCRIPTION
REM
REM  THIS FILE CAN NOT BE COPIED AND/OR DISTRIBUTED WITHOUT EXPRESS
REM  PERMISSION FROM FORCE DIMENSION.

REM This script builds versioning info and applies it to various use cases:
REM case 0 - compose version string (-PdMmprefv)
REM case 1 - target is a header file, update or generate
REM case 2 - no target is given, create versioned <file> for each <file>.ver
REM case 3 - target is a file, version all fields
REM case 4 - target is a folder, recursively version all files within

set invocationDir=%cd%
set scriptDir=%~dp0
set versionFile=%1
set target=%2

REM if version file does not exist, stop
if not exist %versionFile% (
    echo "%versionFile% not found"
    exit /b -1
)

REM retrieve version info
for /F "usebackq delims== tokens=2" %%i in (`findstr PROJECT %versionFile%`) do set PROJECT=%%i
for /F "usebackq delims== tokens=2" %%i in (`findstr PREFIX %versionFile%`) do set PREFIX=%%i
for /F "usebackq delims== tokens=2" %%i in (`findstr DESCRIPTION %versionFile%`) do set DESCRIPTION=%%i
for /F "usebackq delims== tokens=2" %%i in (`findstr MAJOR %versionFile%`) do set MAJOR=%%i
for /F "usebackq delims== tokens=2" %%i in (`findstr MINOR %versionFile%`) do set MINOR=%%i
for /F "usebackq delims== tokens=2" %%i in (`findstr PATCH %versionFile%`) do set PATCH=%%i
for /F "usebackq delims== tokens=2" %%i in (`findstr PRERELEASE %versionFile%`) do set PRERELEASE=%%i
for /F "usebackq delims== tokens=2" %%i in (`findstr METADATA %versionFile%`) do set METADATA=%%i
for /F "usebackq delims== tokens=2" %%i in (`findstr REVISION %versionFile%`) do set REVISION=%%i
if [%PREFIX%] == [] (
    set PREFIX=%PROJECT%_
    call :upper PREFIX
)
if [%METADATA%] == [] (
    for /F "usebackq tokens=1" %%i in (`"git describe --always --dirty --exclude "*""`) do set METADATA=+g%%i
    if !ERRORLEVEL! neq 0 (
        set METADATA=
    )
)
if [%REVISION%] == [] (
    for /F "usebackq tokens=1" %%i in (`"git describe --always --exclude "*""`) do set REVISION=0x%%i
    if !ERRORLEVEL! neq 0 (
        set REVISION=0x0000000
    )
)
set VERSION_FULL=%MAJOR%.%MINOR%.%PATCH%%PRERELEASE%%METADATA%
if [%PRERELEASE%] == [] (
    set VERSION=%MAJOR%.%MINOR%.%PATCH%
) else (
    set VERSION=%VERSION_FULL%
)

REM case 0 - composition
if "%target:~0,1%" == "-" (
    for /l %%i in (1,1,32) do (
        if "!target:~%%i,1!" == "M" (
            echo|set /p="%MAJOR%
        ) else (
        if "!target:~%%i,1!" == "P" (
            echo|set /p="%PROJECT%
        ) else (
        if "!target:~%%i,1!" == "d" (
            echo|set /p="%DESCRIPTION%
        ) else (
        if "!target:~%%i,1!" == "m" (
            echo|set /p="%MINOR%
        ) else (
        if "!target:~%%i,1!" == "p" (
            echo|set /p="%PATCH%
        ) else (
        if "!target:~%%i,1!" == "r" (
            echo|set /p="%PRERELEASE%
        ) else (
        if "!target:~%%i,1!" == "e" (
            echo|set /p="%METADATA%
        ) else (
        if "!target:~%%i,1!" == "v" (
            echo|set /p="%VERSION%
            exit /b 0
        ) else (
        if "!target:~%%i,1!" == "f" (
            echo|set /p="%VERSION_FULL%
            exit /b 0
        ) else (
        if "!target:~%%i,1!" == "" (
            goto break
        ) else (
            echo|set /p="!target:~%%i,1!"
        ))))))))))
    )
    :break
    exit /b 0
)

echo versioning %PROJECT% %VERSION%

REM case 1 - target is a header file, update or generate
for %%i in ("%target%") do (
    set targetFolder=%%~pi
    set targetFilename=%%~ni
    set targetExtension=%%~xi
)
if "!targetExtension!" == ".h" (
    if not exist !targetFolder! (
        md !targetFolder!
    )
    if exist !target! (
        for /F usebackq^ delims^=^"^ tokens^=2 %%j in (`findstr VERSION_FULL !target!`) do set _VERSION_FULL=%%j
        if "!_VERSION_FULL!" == "!VERSION_FULL!" (
            set generateFile=
        ) else (
            set generateFile=y
        )
    ) else (
        set generateFile=y
    )
    if defined generateFile (
        echo // version info                                   > !target!
        echo #define !PREFIX!DESCRIPTION     "!DESCRIPTION!"  >> !target!
        echo #define !PREFIX!VERSION_MAJOR   !MAJOR!          >> !target!
        echo #define !PREFIX!VERSION_MINOR   !MINOR!          >> !target!
        echo #define !PREFIX!VERSION_PATCH   !PATCH!          >> !target!
        echo #define !PREFIX!PRERELEASE      "!PRERELEASE!"   >> !target!
        echo #define !PREFIX!METADATA        "!METADATA!"     >> !target!
        echo #define !PREFIX!VERSION         "!VERSION!"      >> !target!
        echo #define !PREFIX!VERSION_FULL    "!VERSION_FULL!" >> !target!
        echo #define !PREFIX!REVISION        !REVISION!       >> !target!        
    )
    exit /b 0
)

REM case 2 - no target is given, create versioned <file> for each <file>.ver
if "%target%" == "" (
    echo looking for %invocationDir%  
    for %%f in (%invocationDir%\*.ver) do (
        set file=%%~nf
        copy %%f !file!
        %scriptDir%\sed -i ^
        -e "s/$PROJECT/%PROJECT%/g" ^
        -e "s/$PREFIX/%PREFIX%/g" ^
        -e "s/$DESCRIPTION/%DESCRIPTION%/g" ^
        -e "s/$VERSION_FULL/%VERSION_FULL%/g" ^
        -e "s/$VERSION/%VERSION%/g" ^
        -e "s/$MAJOR/%MAJOR%/g" ^
        -e "s/$MINOR/%MINOR%/g" ^
        -e "s/$PATCH/%PATCH%/g" ^
        -e "s/$PRERELEASE/%PRERELEASE%/g" ^
        -e "s/$METADATA/%METADATA%/g" ^
        -e "s/$REVISION/%REVISION%/g" ^
        !file!
    )
)

REM case 3 - target is a file, version all fields
for %%t in (%target%) do if not exist %%~st\NUL (
        %scriptDir%\sed -i ^
        -e "s/$PROJECT/%PROJECT%/g" ^
        -e "s/$PREFIX/%PREFIX%/g" ^
        -e "s/$DESCRIPTION/%DESCRIPTION%/g" ^
        -e "s/$VERSION_FULL/%VERSION_FULL%/g" ^
        -e "s/$VERSION/%VERSION%/g" ^
        -e "s/$MAJOR/%MAJOR%/g" ^
        -e "s/$MINOR/%MINOR%/g" ^
        -e "s/$PATCH/%PATCH%/g" ^
        -e "s/$PRERELEASE/%PRERELEASE%/g" ^
        -e "s/$METADATA/%METADATA%/g" ^
        -e "s/$REVISION/%REVISION%/g" ^
        %%t
)

REM case 4 - target is a folder, recursively version all files within
for %%t in (%target%) do if exist %%~st\NUL (
    for /R %target% %%f in (*) do (
        %scriptDir%\sed -i ^
        -e "s/$PROJECT/%PROJECT%/g" ^
        -e "s/$PREFIX/%PREFIX%/g" ^
        -e "s/$DESCRIPTION/%DESCRIPTION%/g" ^
        -e "s/$VERSION_FULL/%VERSION_FULL%/g" ^
        -e "s/$VERSION/%VERSION%/g" ^
        -e "s/$MAJOR/%MAJOR%/g" ^
        -e "s/$MINOR/%MINOR%/g" ^
        -e "s/$PATCH/%PATCH%/g" ^
        -e "s/$PRERELEASE/%PRERELEASE%/g" ^
        -e "s/$METADATA/%METADATA%/g" ^
        -e "s/$REVISION/%REVISION%/g" ^
        %%f
    )
)


REM upper case function
:upper
set %~1=!%~1:a=A!
set %~1=!%~1:b=B!
set %~1=!%~1:c=C!
set %~1=!%~1:d=D!
set %~1=!%~1:e=E!
set %~1=!%~1:f=F!
set %~1=!%~1:g=G!
set %~1=!%~1:h=H!
set %~1=!%~1:i=I!
set %~1=!%~1:j=J!
set %~1=!%~1:k=K!
set %~1=!%~1:l=L!
set %~1=!%~1:m=M!
set %~1=!%~1:n=N!
set %~1=!%~1:o=O!
set %~1=!%~1:p=P!
set %~1=!%~1:q=Q!
set %~1=!%~1:r=R!
set %~1=!%~1:s=S!
set %~1=!%~1:t=T!
set %~1=!%~1:u=U!
set %~1=!%~1:v=V!
set %~1=!%~1:w=W!
set %~1=!%~1:x=X!
set %~1=!%~1:y=Y!
set %~1=!%~1:z=Z!
exit /b 0

endlocal
