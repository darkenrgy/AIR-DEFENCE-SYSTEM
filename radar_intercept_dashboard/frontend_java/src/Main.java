import javafx.application.Application;
import javafx.scene.Scene;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.control.*;
import javafx.scene.layout.*;
import javafx.stage.Stage;
import javafx.animation.AnimationTimer;
import javafx.scene.paint.Color;
import javafx.scene.text.Font;
import javafx.geometry.Insets;
import javafx.geometry.Pos;

import com.google.gson.*;
import java.net.http.*;
import java.net.URI;
import java.time.Duration;
import java.util.*;
import java.util.concurrent.atomic.AtomicLong;

public class Main extends Application {
    private static final String BACKEND = "http://127.0.0.1:8000/objects";
    private static final String START_SCAN = "http://127.0.0.1:8000/start_scan";
    private static final String STOP_SCAN  = "http://127.0.0.1:8000/stop_scan";
    private static final String CREATE_MANUAL = "http://127.0.0.1:8000/create_manual";
    private static final String PREDICT = "http://127.0.0.1:8000/predict_intercept/";
    private static final String STATS = "http://127.0.0.1:8000/stats";
    private static final String ADD_SENSOR = "http://127.0.0.1:8000/add_sensor";
    private static final String MULTI_SENSOR = "http://127.0.0.1:8000/simulate_multi_sensor";

    private HttpClient client;
    private Gson gson = new Gson();

    private static class Obj {
        int id;
        String type;
        double x,y;
        double vx,vy;
        double cross_section;
        long lastSync;
        List<double[]> interceptPath = new ArrayList<>();
        double trackerX=0, trackerY=0;
        boolean trackerActive=false;
        int trackerIndex=0;
        double trackerSpeed=300.0;
    }

    private final Map<Integer, Obj> objects = new HashMap<>();
    private final AtomicLong lastPoll = new AtomicLong(0);
    // stats fields
    private int sensorCount = 1;
    private int interceptorsSent = 0;

    @Override
    public void start(Stage stage) {
        client = HttpClient.newBuilder().connectTimeout(Duration.ofSeconds(2)).build();

        BorderPane root = new BorderPane();
        // left: controls & stats
        VBox controls = new VBox(8);
        controls.setPadding(new Insets(10));
        controls.setStyle("-fx-background-color: #001100;");
        Label title = new Label("BLUE WHALE DASHBOARD");
        title.setTextFill(Color.LIME);
        title.setFont(Font.font("Consolas", 16));

        HBox btnRow = new HBox(6);
        Button btnStart = new Button("Start Scan");
        Button btnStop  = new Button("Stop Scan");
        Button btnAddSensor = new Button("Add Sensor");
        btnRow.getChildren().addAll(btnStart, btnStop, btnAddSensor);

        Button btnAddManual = new Button("Add Manual Object");

        ListView<String> objList = new ListView<>();
        objList.setPrefHeight(220);

        // manual inputs
        TextField tfType = new TextField("drone");
        TextField tfPos  = new TextField("-500,400");
        TextField tfVel  = new TextField("120,-30");
        TextField tfCS   = new TextField("1.0");

        // stats panel
        GridPane stats = new GridPane();
        stats.setHgap(6); stats.setVgap(6);
        stats.add(new Label("Radars:"), 0,0);
        Label lblRadars = new Label("1"); lblRadars.setTextFill(Color.LIME);
        stats.add(lblRadars, 1,0);
        stats.add(new Label("Objects:"), 0,1);
        Label lblObjects = new Label("0"); lblObjects.setTextFill(Color.LIME);
        stats.add(lblObjects, 1,1);
        stats.add(new Label("Interceptors Sent:"), 0,2);
        Label lblInter = new Label("0"); lblInter.setTextFill(Color.LIME);
        stats.add(lblInter, 1,2);

        // buttons for selecting interceptor candidate path
        Label lblSelect = new Label("Select interceptor path for object ID:");
        TextField tfSelectId = new TextField();
        Button btnSelectPath = new Button("Send Interceptor to First Candidate");

        btnStart.setOnAction(e -> sendSimplePost(START_SCAN));
        btnStop.setOnAction(e -> sendSimplePost(STOP_SCAN));
        btnAddSensor.setOnAction(e -> {
            sendSimplePost(ADD_SENSOR);
            sensorCount++;
            lblRadars.setText(String.valueOf(sensorCount));
        });

        btnAddManual.setOnAction(e -> {
            try {
                String type = tfType.getText().trim();
                String[] posS = tfPos.getText().split(",");
                String[] velS = tfVel.getText().split(",");
                double px = Double.parseDouble(posS[0].trim());
                double py = Double.parseDouble(posS[1].trim());
                double vx = Double.parseDouble(velS[0].trim());
                double vy = Double.parseDouble(velS[1].trim());
                double cs = Double.parseDouble(tfCS.getText().trim());
                Map<String,Object> body = new HashMap<>();
                body.put("type", type);
                body.put("position", Arrays.asList(px,py));
                body.put("velocity", Arrays.asList(vx,vy));
                body.put("cross_section", cs);
                String json = gson.toJson(body);
                HttpRequest req = HttpRequest.newBuilder()
                    .uri(URI.create(CREATE_MANUAL))
                    .timeout(Duration.ofSeconds(2))
                    .POST(HttpRequest.BodyPublishers.ofString(json))
                    .header("Content-Type","application/json")
                    .build();
                client.sendAsync(req, HttpResponse.BodyHandlers.ofString());
            } catch (Exception ex) { ex.printStackTrace(); }
        });

        btnSelectPath.setOnAction(e -> {
            try {
                int id = Integer.parseInt(tfSelectId.getText().trim());
                // request predict and then increment interceptor count on success
                new Thread(() -> {
                    try {
                        HttpRequest req = HttpRequest.newBuilder().uri(URI.create(PREDICT + id)).timeout(Duration.ofSeconds(1)).POST(HttpRequest.BodyPublishers.noBody()).build();
                        HttpResponse<String> resp = client.send(req, HttpResponse.BodyHandlers.ofString());
                        if (resp.statusCode()==200) {
                            synchronized(objects) {
                                Obj o = objects.get(id);
                                if (o!=null) {
                                    // assume backend sent candidates earlier; send interceptor (visual)
                                    o.trackerActive = true;
                                    o.trackerIndex = 0;
                                    interceptorsSent++;
                                    lblInter.setText(String.valueOf(interceptorsSent));
                                }
                            }
                        }
                    } catch (Exception ex) {}
                }).start();
            } catch (Exception ex) {}
        });

        controls.getChildren().addAll(title, btnRow, new Separator(), new Label("Manual create (type,pos,vx,vy,cs)"), tfType, tfPos, tfVel, tfCS, btnAddManual, new Separator(), new Label("Objects"), objList, new Separator(), stats, new Separator(), lblSelect, tfSelectId, btnSelectPath);

        // center: canvas radar + right: details panel
        Canvas canvas = new Canvas(900,700);
        GraphicsContext gc = canvas.getGraphicsContext2D();
        root.setLeft(controls);
        root.setCenter(canvas);

        Scene scene = new Scene(root, 1300, 720);
        stage.setScene(scene);
        stage.setTitle("Radar Intercept Dashboard v2");
        stage.show();

        // animation timer
        AnimationTimer timer = new AnimationTimer() {
            private long prev = -1;
            @Override
            public void handle(long now) {
                if (prev<0) prev = now;
                double dt = (now - prev) / 1_000_000_000.0;
                prev = now;
                if (now - lastPoll.get() > 350_000_000L) {
                    lastPoll.set(now);
                    pollBackendAsync();
                    requestStatsAsync();
                }
                updateLocal(dt);
                updateTrackers(dt);
                draw(gc);
                // update list view and stats labels
                synchronized(objects) {
                    objList.getItems().clear();
                    for (Obj o: objects.values()) {
                        objList.getItems().add(o.type+" #"+o.id+" cs="+String.format("%.1f", o.cross_section)+" sp="+String.format("%.0f", Math.hypot(o.vx,o.vy)));
                    }
                    lblObjects.setText(String.valueOf(objects.size()));
                }
            }
        };
        timer.start();
    }

    private void sendSimplePost(String url) {
        try {
            HttpRequest req = HttpRequest.newBuilder().uri(URI.create(url)).timeout(Duration.ofSeconds(1)).POST(HttpRequest.BodyPublishers.noBody()).build();
            client.sendAsync(req, HttpResponse.BodyHandlers.ofString());
        } catch (Exception ex) {}
    }

    private void requestStatsAsync() {
        new Thread(() -> {
            try {
                HttpRequest req = HttpRequest.newBuilder().uri(URI.create(STATS)).timeout(Duration.ofSeconds(1)).GET().build();
                HttpResponse<String> resp = client.send(req, HttpResponse.BodyHandlers.ofString());
                if (resp.statusCode()==200) {
                    JsonObject jo = gson.fromJson(resp.body(), JsonObject.class);
                    int sensors = jo.get("sensor_count").getAsInt();
                    int inter = jo.get("interceptors_sent").getAsInt();
                    synchronized(objects) {
                        // update labels via UI thread - but we simply set local fields and let timer update labels
                        // (JavaFX requires Platform.runLater for UI updates; using simple approach here)
                    }
                    sensorCount = sensors;
                    interceptorsSent = inter;
                }
            } catch (Exception ex) {}
        }).start();
    }

    private void pollBackendAsync() {
        new Thread(() -> {
            try {
                HttpRequest req = HttpRequest.newBuilder().uri(URI.create(BACKEND)).timeout(Duration.ofSeconds(1)).GET().build();
                HttpResponse<String> resp = client.send(req, HttpResponse.BodyHandlers.ofString());
                if (resp.statusCode()==200) {
                    JsonObject jo = gson.fromJson(resp.body(), JsonObject.class);
                    JsonArray arr = jo.getAsJsonArray("objects");
                    long now = System.currentTimeMillis();
                    synchronized(objects) {
                        Set<Integer> present = new HashSet<>();
                        for (JsonElement e: arr) {
                            JsonObject o = e.getAsJsonObject();
                            int id = o.get("id").getAsInt();
                            present.add(id);
                            Obj obj = objects.getOrDefault(id, new Obj());
                            obj.id = id;
                            obj.type = o.get("type").getAsString();
                            JsonArray pos = o.getAsJsonArray("position");
                            obj.x = pos.get(0).getAsDouble();
                            obj.y = pos.get(1).getAsDouble();
                            JsonArray vel = o.getAsJsonArray("velocity");
                            obj.vx = vel.get(0).getAsDouble();
                            obj.vy = vel.get(1).getAsDouble();
                            obj.cross_section = o.get("cross_section").getAsDouble();
                            obj.lastSync = now;
                            if (!obj.trackerActive) { obj.trackerX = 0; obj.trackerY = 0; }
                            objects.put(id, obj);
                        }
                        objects.keySet().removeIf(k -> !present.contains(k));
                    }
                    // request intercept candidates for each
                    for (Integer id: objects.keySet()) requestInterceptAsync(id);
                }
            } catch (Exception ex) {}
        }).start();
    }

    private void requestInterceptAsync(int id) {
        new Thread(() -> {
            try {
                HttpRequest req = HttpRequest.newBuilder().uri(URI.create(PREDICT + id)).timeout(Duration.ofSeconds(1)).POST(HttpRequest.BodyPublishers.noBody()).build();
                HttpResponse<String> resp = client.send(req, HttpResponse.BodyHandlers.ofString());
                if (resp.statusCode()==200) {
                    JsonObject jo = gson.fromJson(resp.body(), JsonObject.class);
                    JsonArray arr = jo.getAsJsonArray("candidates");
                    List<double[]> path = new ArrayList<>();
                    for (JsonElement e: arr) {
                        JsonObject c = e.getAsJsonObject();
                        JsonArray p = c.getAsJsonArray("point");
                        double px = p.get(0).getAsDouble();
                        double py = p.get(1).getAsDouble();
                        path.add(new double[]{px,py});
                    }
                    synchronized(objects) {
                        Obj o = objects.get(id);
                        if (o!=null) {
                            o.interceptPath = path;
                            if (!path.isEmpty()) {
                                o.trackerActive = true;
                                o.trackerIndex = 0;
                            } else {
                                o.trackerActive = false;
                            }
                        }
                    }
                }
            } catch (Exception ex) {}
        }).start();
    }

    private void updateLocal(double dt) {
        synchronized(objects) {
            for (Obj o: objects.values()) {
                o.x += o.vx * dt;
                o.y += o.vy * dt;
            }
        }
    }

    private void updateTrackers(double dt) {
        synchronized(objects) {
            for (Obj o: objects.values()) {
                if (!o.trackerActive || o.interceptPath.isEmpty()) continue;
                int idx = Math.min(o.trackerIndex, o.interceptPath.size()-1);
                double[] tgt = o.interceptPath.get(idx);
                double dx = tgt[0] - o.trackerX;
                double dy = tgt[1] - o.trackerY;
                double dist = Math.hypot(dx, dy);
                double maxmove = o.trackerSpeed * dt;
                if (dist < 2.0) {
                    if (o.trackerIndex < o.interceptPath.size()-1) o.trackerIndex++;
                } else {
                    double ux = dx / dist, uy = dy / dist;
                    double move = Math.min(maxmove, dist);
                    o.trackerX += ux * move;
                    o.trackerY += uy * move;
                }
            }
        }
    }

    private void draw(GraphicsContext gc) {
        gc.setFill(Color.BLACK);
        gc.fillRect(0,0,900,700);
        gc.setStroke(Color.LIME);
        gc.setLineWidth(1.2);
        double cx = 450, cy = 350;
        for (int r=100; r<=400; r+=100) gc.strokeOval(cx-r, cy-r, r*2, r*2);
        gc.strokeLine(cx-450, cy, cx+450, cy);
        gc.strokeLine(cx, cy-300, cx, cy+300);
        gc.setFill(Color.LIME);
        gc.fillText("RADAR - Objects: " + objects.size(), 10, 20);
        synchronized(objects) {
            for (Obj o: objects.values()) {
                double px = o.x/4.0 + cx;
                double py = cy - o.y/4.0;
                if ("missile".equals(o.type)) gc.fillOval(px-6, py-6, 12, 12);
                else if ("drone".equals(o.type)) gc.fillRect(px-5, py-5, 10, 10);
                else gc.strokeOval(px-5, py-5, 10, 10);
                gc.fillText(o.type+" #"+o.id, px+8, py);
                if (o.interceptPath!=null && !o.interceptPath.isEmpty()) {
                    gc.setLineDashes(6,6);
                    double sx = px, sy = py;
                    gc.setStroke(Color.rgb(0,255,100,0.8));
                    for (double[] p: o.interceptPath) {
                        double tx = p[0]/4.0 + cx;
                        double ty = cy - p[1]/4.0;
                        gc.strokeLine(sx, sy, tx, ty);
                        gc.fillOval(tx-4, ty-4, 8, 8);
                        sx = tx; sy = ty;
                    }
                    gc.setLineDashes(null);
                }
                if (o.trackerActive) {
                    double tx = o.trackerX/4.0 + cx;
                    double ty = cy - o.trackerY/4.0;
                    gc.setFill(Color.YELLOW);
                    gc.fillOval(tx-5, ty-5, 10, 10);
                    gc.fillText("INT", tx+6, ty);
                }
            }
        }
    }

    public static void main(String[] args) {
        launch(args);
    }
}
