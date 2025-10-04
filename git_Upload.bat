@echo off
REM Batch file to commit and push selected KiCad files

REM Stage only selected KiCad files
git add .

REM Check if commit message is provided as argument
IF "%1"=="" (
    set /p msg="Enter commit message: "
) ELSE (
    set msg=%1
)

git commit -m "%msg%"
git push origin main

echo Git update complete!
pause
