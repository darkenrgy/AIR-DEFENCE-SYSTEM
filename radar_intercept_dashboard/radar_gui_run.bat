@echo off
set JAVA_FX=D:\javafx-sdk-25.0.1\lib
cd /d %~dp0\frontend_java
echo Compiling...
javac -cp ".;gson.jar" --module-path %JAVA_FX% --add-modules javafx.controls,javafx.fxml src\Main.java
if %errorlevel% neq 0 (
  echo Compile failed
  pause
  exit /b
)
echo Running...
java --enable-native-access=javafx.graphics -cp ".;gson.jar;src" --module-path %JAVA_FX% --add-modules javafx.controls,javafx.fxml Main
pause
