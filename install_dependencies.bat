@echo off
set PLATFORM=x64
set PLATFORM_X=64
set SFML_VERSION=2.5.1
set SFML_DIR=SFML-%SFML_VERSION%
set SFML_ZIP=SFML-%SFML_VERSION%-windows-vc15-%PLATFORM_X%-bit.zip
set SFML_URL=https://www.sfml-dev.org/files/%SFML_ZIP%
set "SFML_PROJECTS=FactorioBlueprints"

@REM Check if curl or wget is available
where curl >nul 2>&1
if %ERRORLEVEL%==0 (
    set DOWNLOAD_CMD=curl -LO
) else (
    where wget >nul 2>&1
    if %ERRORLEVEL%==0 (
        set DOWNLOAD_CMD=wget
    ) else (
        echo "Neither curl nor wget is available. Please install one of them."
        exit /b 1
    )
)

@REM Download SFML if the zip doesn't exist
if not exist %SFML_ZIP% (
    echo :: Downloading SFML...
    %DOWNLOAD_CMD% %SFML_URL%
)

@REM Extract SFML if the directory doesn't exist
if not exist %SFML_DIR% (
    echo :: Extracting SFML...
    tar -xf %SFML_ZIP% || (
        echo "Extraction failed. Please ensure tar is installed or extract manually."
        exit /b 1
    )
)

@REM Create required dependency directories if they don't exist
echo :: Ensuring dependencies directory structure...
if not exist lib mkdir dependencies\SFML\lib
if not exist include mkdir dependencies\SFML\include

@REM Copy SFML include / lib files to dependencies
echo :: Copying SFML includes / lib files to dependencies...
xcopy /E /I /Y %SFML_DIR%\lib dependencies\SFML\lib | find "File(s) copied"
xcopy /E /I /Y %SFML_DIR%\include dependencies\SFML\include | find "File(s) copied"

@REM Copy SFML DLLs to required projects output directories
echo :: Copying SFML DLLs to project output directories...
for %%i in (%SFML_PROJECTS%) do (

    @REM Create required directories if they don't exist
    echo :: Creating output directories for project %%i...
    if not exist bin\%%i\%PLATFORM%\output\Debug mkdir bin\%%i\%PLATFORM%\output\Debug
    if not exist bin\%%i\%PLATFORM%\output\Release mkdir bin\%%i\%PLATFORM%\output\Release
    
    @REM Copy all *-d-2.dll to Debug output, and *-2.dll to Release output
    for %%j in (%SFML_DIR%\bin\*-2.dll) do (
        echo %%~nxj | findstr /i /c:"-d-2.dll" >nul
        if not errorlevel 1 (
            copy /Y "%%j" bin\%%i\%PLATFORM%\output\Debug >nul
        ) else (
            copy /Y "%%j" bin\%%i\%PLATFORM%\output\Release >nul
        )
    )
)

@REM Cleanup extracted SFML zip files
echo :: Cleaning up extracted SFML files...
if exist %SFML_DIR% rmdir /s /q %SFML_DIR%
if exist %SFML_ZIP% del %SFML_ZIP%

@REM Done
echo SFML setup complete!
