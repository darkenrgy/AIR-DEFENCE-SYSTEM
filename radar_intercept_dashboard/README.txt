Radar Intercept Dashboard v2 (Java + Python)
--------------------------------------------
This package contains a backend (FastAPI) with multi-sensor fusion example and a JavaFX frontend
which shows scanner + interceptor UI in a single dashboard. The GUI includes controls to start/stop scanning,
add sensors, add manual objects, select interceptor paths, and a stats panel showing number of radars,
number of objects, and interceptors sent.

Quick start (Windows):
1) Backend:
   cd backend_python
   python -m venv venv
   .\venv\Scripts\Activate.ps1
   pip install -r requirements.txt
   uvicorn main:app --reload
2) Frontend:
   place gson jar at frontend_java/gson.jar (download gson-2.10.1.jar)
   edit radar_gui_run.bat JAVA_FX path if needed
   run radar_gui_run.bat

Notes:
- gson.jar is not included; download gson-2.10.1.jar and place as frontend_java/gson.jar
- JavaFX SDK path must be set to your local installation
