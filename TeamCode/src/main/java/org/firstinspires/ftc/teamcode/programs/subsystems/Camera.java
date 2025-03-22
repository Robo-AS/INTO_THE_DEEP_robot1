package org.firstinspires.ftc.teamcode.programs.subsystems;

import java.io.*;

public class Camera {
    private Process process;
    private BufferedReader reader;
    private String lastDetection = "NOT_DETECTED";
    public Camera() {
        try {
            ProcessBuilder pb = new ProcessBuilder("python", "teamcode/pythonScripts/script.py");
            pb.redirectErrorStream(true);
            process = pb.start();

            reader = new BufferedReader(new InputStreamReader(process.getInputStream()));
            new Thread(() -> {
                try {
                    String line;
                    while ((line = reader.readLine()) != null) {
                        lastDetection = line.trim();
                    }
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }).start();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public boolean isBlueObjectDetected() {
        return lastDetection.equals("DETECTED");
    }

    public void stop() {
        if (process != null) {
            process.destroy();
        }
    }
}
