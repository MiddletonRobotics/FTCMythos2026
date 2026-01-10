package org.firstinspires.ftc.teamcode.utilities;

import android.util.Log;

import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;
import java.util.Map;

public class TelemetryData {
    private final Telemetry telemetry;
    private final TelemetryManager telemetryManager;

    private final Map<String, String> dataMap = new HashMap<>();

    public TelemetryData(TelemetryManager telemetryManager, Telemetry telemetry) {
        this.telemetryManager = telemetryManager;
        this.telemetry = telemetry;
    }

    public void addData(String caption, Object value) {
        dataMap.put(caption, value.toString());
    }

    public void update() {
        telemetry.update();
        for (Map.Entry<String, String> entry : dataMap.entrySet()) {
            Log.v(entry.getKey(), entry.getValue());
        }

        dataMap.clear();
    }
}