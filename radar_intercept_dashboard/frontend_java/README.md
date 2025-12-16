Frontend JavaFX - Radar Intercept Dashboard (v2)
------------------------------------------------
Requirements:
- JDK 17+
- JavaFX SDK (module path)
- gson jar for JSON parsing (place as frontend_java/gson.jar)

Compile & run (PowerShell):
$JAVA_FX = 'D:\javafx-sdk-25.0.1\lib'
cd frontend_java
javac -cp ".;gson.jar" --module-path $JAVA_FX --add-modules javafx.controls,javafx.fxml src\Main.java
java --enable-native-access=javafx.graphics -cp ".;gson.jar;src" --module-path $JAVA_FX --add-modules javafx.controls,javafx.fxml Main
