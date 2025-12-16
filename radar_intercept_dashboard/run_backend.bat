@echo off
cd /d %~dp0\backend_python
if not exist venv (
  python -m venv venv
  call venv\Scripts\Activate.bat
  pip install -r requirements.txt
) else (
  call venv\Scripts\Activate.bat
)
uvicorn main:app --reload
