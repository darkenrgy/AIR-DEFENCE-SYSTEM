Radar Intercept Dashboard - Backend (v2)
----------------------------------------
Adds sensor_integration simulation and stats endpoint.

Quick start (Windows PowerShell):
1. cd backend_python
2. python -m venv venv
3. .\venv\Scripts\Activate.ps1
4. pip install -r requirements.txt
5. uvicorn main:app --reload

Endpoints:
- GET  /objects
- GET  /stats
- POST /start_scan
- POST /stop_scan
- POST /create_manual
- POST /predict_intercept/{obj_id}
- POST /add_sensor
- POST /simulate_multi_sensor -> simulate combining data from multiple sensors
