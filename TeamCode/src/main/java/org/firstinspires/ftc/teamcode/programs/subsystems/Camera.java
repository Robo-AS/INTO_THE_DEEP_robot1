package org.firstinspires.ftc.teamcode.programs.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.*;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Camera {
    private boolean sampleFlag = false;
    private double x = 0, y = 0, focalLength = 720, sampleAngle;
    private double sampleWidth = 3.8, width;
    private Telemetry telemetry;
    private OpenCvWebcam webcam;

    ColorDetectionPipeline pipeline = new ColorDetectionPipeline();

    public Camera(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName);

        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera error: ", errorCode);
                telemetry.update();
            }
        });
    }

    public void cameraLoop(Mecanum mecanum, Claw claw) {
        if (sampleFlag) {
            double angle = pipeline.getSampleTilt();
            claw.adjustForSample(angle);
        }
    }

    public enum Position {
        LEFT, MIDDLE, RIGHT
    }

    public Position getPosition() {
        if (x >= 400)
            return Position.RIGHT;
        else if (x < 400 && x > 150)
            return Position.MIDDLE;
        else
            return Position.LEFT;
    }

    public double getDistance() {
        return (sampleWidth * focalLength) / width;
    }

    private class ColorDetectionPipeline extends OpenCvPipeline {
        private Mat hsvMat = new Mat();
        private Mat mask = new Mat();
        private Mat hierarchy = new Mat();
        private Scalar lowerBound = new Scalar(100, 150, 70);
        private Scalar upperBound = new Scalar(125, 255, 255);

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
            Core.inRange(hsvMat, lowerBound, upperBound, mask);

            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            if (!contours.isEmpty()) {
                MatOfPoint largestContour = contours.get(0);
                double maxArea = 0;

                for (MatOfPoint contour : contours) {
                    double area = Imgproc.contourArea(contour);
                    if (area > maxArea) {
                        maxArea = area;
                        largestContour = contour;
                    }
                }

                if (largestContour != null) {
                    sampleFlag = true;
                    width = calculateWidth(largestContour);

                    Moments moments = Imgproc.moments(largestContour);
                    x = moments.get_m10() / moments.get_m00();
                    y = moments.get_m01() / moments.get_m00();

                    RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(largestContour.toArray()));
                    sampleAngle = rect.angle;

                    if(rect.size.width < rect.size.height)
                        sampleAngle += 90;
                }

                Rect boundingRect = Imgproc.boundingRect(largestContour);
                Imgproc.rectangle(input, boundingRect, new Scalar(0, 255, 0), 2);

                Point center = new Point(boundingRect.x + boundingRect.width / 2.0, boundingRect.y + boundingRect.height / 2.0);
                Imgproc.circle(input, center, 5, new Scalar(0, 255, 0), -1);
                Imgproc.putText(input, "Target color found", new Point(center.x - 20, center.y - 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
            }

            return input;
        }

        private long calculateWidth(MatOfPoint contour) {
            Rect boundingRect = Imgproc.boundingRect(contour);
            return boundingRect.width;
        }

        public double getSampleTilt(){
            return sampleAngle;
        }
    }
}
