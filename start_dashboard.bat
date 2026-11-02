@echo off
cd /d "%~dp0"
echo Starting VENOTRACE Research Dashboard...
if exist .venv\Scripts\python.exe (
    .venv\Scripts\python.exe -m uvicorn main:app --host 127.0.0.1 --port 8000
) else (
    echo Virtual environment not found. Running with system python:
    python -m uvicorn main:app --host 127.0.0.1 --port 8000
)
pause
