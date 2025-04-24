package org.firstinspires.ftc.teamcode.programs.subsystems;

import org.opencv.core.MatOfDouble;
import org.opencv.core.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.teamcode.programs.utils.Robot;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.RotatedRect;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point3;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.*;

@TeleOp
public class Camera extends OpMode {
    double cX = 0, cY = 0, width = 0, angle = 0;
    public static final double objectWidthInRealWorldUnits = 3.5;
    OpenCvWebcam webcam = null;
    colorDetection pipeline;
    public Mat cameraMatrix;
    public MatOfDouble distCoeffs;

    @Override
    public void init() {
        cameraMatrix = new Mat(3, 3, CvType.CV_64F);
        distCoeffs = new MatOfDouble(0.82856579, -29.9885279, 0.01559277, 0.00330627, 367.63255);

        cameraMatrix.put(0, 0, new double[]{
                1941.37278, 0, 626.320287,
                0, 1940.45508, 406.938078,
                0, 0, 1
        });

        WebcamName webcamName = Robot.getInstanceHardwareMap().get(WebcamName.class, "webcam1");
        int cameraID = Robot.getInstanceHardwareMap().appContext.getResources().getIdentifier("cameraID", "id", Robot.getInstanceHardwareMap().appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraID);
        pipeline = new colorDetection();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });
    }

    @Override
    public void loop() {}

    public double sampleAngle() {
        double normalized = angle / 180.0;
        return Range.clip(normalized, 0.0, 1.0);
    }

    class colorDetection extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            Mat BlueMask = preprocessFrame(input);

            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(BlueMask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            MatOfPoint largestContour = findLargestContour(contours);

            if (largestContour != null) {
                Moments moments = Imgproc.moments(largestContour);
                cX = moments.get_m10() / moments.get_m00();
                cY = moments.get_m01() / moments.get_m00();

                Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(0, 255, 255), 2);
                width = calculateWidth(largestContour);

                angle = getAngleWithPnP(largestContour);

                Imgproc.putText(input, "Width: " + (int) width + " px", new Point(cX + 5, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                Imgproc.putText(input, "Distance: " + String.format("%.2f", getDistance(width)) + " in", new Point(cX + 5, cY + 40), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                Imgproc.putText(input, "Angle: " + String.format("%.2f", angle) + " deg", new Point(cX + 5, cY + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);
            }

            return input;
        }

        private Mat preprocessFrame(Mat frame) {
            Mat hsvFrame = new Mat();
            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);
            Scalar lowerBlue = new Scalar(0, 100, 100);
            Scalar upperBlue = new Scalar(50, 255, 255);

            Mat BlueMask = new Mat();
            Core.inRange(hsvFrame, lowerBlue, upperBlue, BlueMask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(BlueMask, BlueMask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(BlueMask, BlueMask, Imgproc.MORPH_CLOSE, kernel);

            return BlueMask;
        }
    }

    private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
        double maxArea = 0;
        MatOfPoint largestContour = null;
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > maxArea) {
                maxArea = area;
                largestContour = contour;
            }
        }
        return largestContour;
    }

    private double calculateWidth(MatOfPoint contour) {
        Rect boundingRect = Imgproc.boundingRect(contour);
        return boundingRect.width;
    }

    public static double getDistance(double width) {
        return (objectWidthInRealWorldUnits * 720) / width;
    }

    public double getAngleWithPnP(MatOfPoint contour) {
        MatOfPoint2f approx = new MatOfPoint2f();
        Imgproc.approxPolyDP(new MatOfPoint2f(contour.toArray()), approx, 0.02 * Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true), true);

        if (approx.total() != 4) return getAngle(contour);

        Point[] pts = approx.toArray();
        Arrays.sort(pts, Comparator.comparingDouble(p -> p.y + p.x));
        Point tl = pts[0], br = pts[3];
        Arrays.sort(pts, Comparator.comparingDouble(p -> p.y - p.x));
        Point tr = pts[0], bl = pts[3];

        MatOfPoint2f imagePoints = new MatOfPoint2f(tl, tr, br, bl);

        List<Point3> objPoints = Arrays.asList(
                new Point3(0, 0, 0),
                new Point3(3.5, 0, 0),
                new Point3(3.5, 2.0, 0),
                new Point3(0, 2.0, 0)
        );
        MatOfPoint3f objectPoints = new MatOfPoint3f();
        objectPoints.fromList(objPoints);

        Mat rvec = new Mat(), tvec = new Mat();

        boolean solved = Calib3d.solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);
        if (!solved) return getAngle(contour);

        Mat rotMat = new Mat();
        Calib3d.Rodrigues(rvec, rotMat);

        double yaw = Math.atan2(rotMat.get(1, 0)[0], rotMat.get(0, 0)[0]);
        double angleDegrees = Math.toDegrees(yaw);

        // Normalize to range [0, 180]
        if (angleDegrees < 0) {
             angleDegrees += 180;
        }
        return angleDegrees;

    }

    public double getAngle(MatOfPoint contour) {
        if (contour == null || contour.toArray().length == 0) return Double.NaN;

        RotatedRect rotatedRect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));
        return Math.abs(rotatedRect.angle);
    }
}
