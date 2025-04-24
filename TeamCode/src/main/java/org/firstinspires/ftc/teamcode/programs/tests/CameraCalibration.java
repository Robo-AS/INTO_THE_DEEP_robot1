package org.firstinspires.ftc.teamcode.programs.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.*;
import org.opencv.calib3d.Calib3d;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class CameraCalibration extends OpMode {
    OpenCvCamera webcam;
    CalibrationPipeline pipeline;

    final Size patternSize = new Size(9, 6);  // Use a 9x6 chessboard
    final float squareSize = 1.0f;  // Arbitrary units (can be inches, cm, etc)

    List<Mat> imagePoints = new ArrayList<>();
    List<Mat> objectPoints = new ArrayList<>();

    boolean calibrationComplete = false;

    @Override
    public void init() {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam1");
        int camMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, camMonitorViewId);
        pipeline = new CalibrationPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });
    }

    @Override
    public void loop() {
        telemetry.addLine("Press A to capture frame for calibration");
        telemetry.addLine("Press B to calibrate");
        telemetry.addData("Captured Frames", imagePoints.size());

        if (gamepad1.a && pipeline.lastCorners != null) {
            imagePoints.add(pipeline.lastCorners.clone());

            MatOfPoint3f objPts = new MatOfPoint3f();
            List<Point3> corners = new ArrayList<>();
            for (int i = 0; i < patternSize.height; i++) {
                for (int j = 0; j < patternSize.width; j++) {
                    corners.add(new Point3(j * squareSize, i * squareSize, 0));
                }
            }
            objPts.fromList(corners);
            objectPoints.add(objPts);

            telemetry.addLine("Frame captured!");
        }

        if (gamepad1.b && !calibrationComplete && imagePoints.size() >= 10) {
            Mat cameraMatrix = Mat.eye(3, 3, CvType.CV_64F);
            Mat distCoeffs = Mat.zeros(8, 1, CvType.CV_64F);
            List<Mat> rvecs = new ArrayList<>();
            List<Mat> tvecs = new ArrayList<>();

            double error = Calib3d.calibrateCamera(objectPoints, imagePoints, pipeline.imageSize,
                    cameraMatrix, distCoeffs, rvecs, tvecs);

            telemetry.addLine("Calibration Complete!");
            telemetry.addData("Reprojection Error", error);
            telemetry.addData("Camera Matrix", cameraMatrix.dump());
            telemetry.addData("Distortion Coeffs", distCoeffs.dump());

            calibrationComplete = true;
        }

        telemetry.update();
    }

    class CalibrationPipeline extends OpenCvPipeline {
        Mat gray = new Mat();
        MatOfPoint2f corners = new MatOfPoint2f();
        Mat lastCorners;
        Size imageSize = new Size();

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, gray, Imgproc.COLOR_BGR2GRAY);
            imageSize = gray.size();

            boolean found = Calib3d.findChessboardCorners(gray, patternSize, corners,
                    Calib3d.CALIB_CB_ADAPTIVE_THRESH + Calib3d.CALIB_CB_NORMALIZE_IMAGE + Calib3d.CALIB_CB_FAST_CHECK);

            if (found) {
                Calib3d.drawChessboardCorners(input, patternSize, corners, found);
                lastCorners = corners.clone();
            }

            return input;
        }
    }
}
