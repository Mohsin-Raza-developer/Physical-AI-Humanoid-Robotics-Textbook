@echo off
echo ========================================
echo   Starting All Servers
echo ========================================
echo.

echo [1/3] Starting Frontend (Port 3000)...
start "Frontend - Port 3000" cmd /k "npm run start"
timeout /t 3 /nobreak > nul

echo [2/3] Starting Auth Backend (Port 3002)...
start "Auth Backend - Port 3002" cmd /k "cd auth-backend && npm run dev"
timeout /t 3 /nobreak > nul

echo [3/3] Starting Chatbot Backend (Port 8000)...
start "Chatbot Backend - Port 8000" cmd /k "cd app && uvicorn main:app --reload --port 8000"

echo.
echo ========================================
echo   All Servers Started!
echo ========================================
echo.
echo Open these URLs in browser:
echo   - Frontend: http://localhost:3000
echo   - Auth API: http://localhost:3002
echo   - Chatbot API: http://localhost:8000
echo.
echo Press any key to close this window...
pause > nul
