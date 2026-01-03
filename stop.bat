@echo off
echo ========================================
echo   Stopping All Servers
echo ========================================
echo.

echo Killing Node.js processes (Frontend + Auth Backend)...
taskkill /F /IM node.exe 2>nul
if %errorlevel% == 0 (
    echo   - Node.js processes stopped ✓
) else (
    echo   - No Node.js processes found
)

echo.
echo Killing Python processes (Chatbot Backend)...
taskkill /F /IM python.exe 2>nul
if %errorlevel% == 0 (
    echo   - Python processes stopped ✓
) else (
    echo   - No Python processes found
)

echo.
echo ========================================
echo   All Servers Stopped!
echo ========================================
echo.
pause
