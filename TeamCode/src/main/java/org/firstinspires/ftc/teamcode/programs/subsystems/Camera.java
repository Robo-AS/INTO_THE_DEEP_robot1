package org.firstinspires.ftc.teamcode.programs.subsystems;

import org.ejml.dense.row.misc.ImplCommonOps_DDMA;
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
    List<MatOfPoint> blues = new ArrayList<>(), reds = new ArrayList<>(), yellows = new ArrayList<>(), contours = new ArrayList<>();
    public double cX = 0, cY = 0, width = 0, angle = 0, height = 0;
    public static final double objectWidthInRealWorldUnits = 3.5;
    OpenCvWebcam webcam = null;
    colorDetection pipeline;
    public Mat cameraMatrix;
    public MatOfDouble distCoeffs;

    public double tx = 0.0, ty = 0.0;
    public static final double CAMERA_OFFSET_CM = 15.5;

    public enum Detection{
        BLUE,
        RED,
        BLUE_YELLOW,
        RED_YELLOW
    }

    private Detection detectionMode = Detection.BLUE_YELLOW;

    public void setDetectionMode(Detection mode) {
        detectionMode = mode;
    }

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
        pipeline = new colorDetection(this);
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
    public void loop() {

    }

    public double sampleAngle() {
        double normalized = angle / 180.0;
        return Range.clip(normalized, 0.0, 1.0);
    }

    class colorDetection extends OpenCvPipeline {
        Camera parent;

        public colorDetection(Camera parent) {
            this.parent = parent;
        }

        @Override
        public Mat processFrame(Mat input) {
            Detection detectionMode = parent.detectionMode;

            Mat BlueMask = preprocessBlue(input);
            Mat RedMask = preprocessRed(input);
            Mat YellowMask = preprocessYellow(input);


            Mat hierarchy = new Mat();
            Imgproc.findContours(BlueMask, blues, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.findContours(RedMask, reds, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.findContours(YellowMask, yellows, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            MatOfPoint maximumBlue = findLargestContour(blues);
            MatOfPoint maximumRed = findLargestContour(reds);
            MatOfPoint maximumYellow = findLargestContour(yellows);

            MatOfPoint largestContour = new MatOfPoint();

            if(detectionMode == Detection.BLUE && maximumBlue != null) {
                largestContour = maximumBlue;
                contours = blues;
            }
            else if(detectionMode == Detection.RED && maximumRed != null)
            {
                largestContour = maximumRed;
                contours = reds;
            }
            else if(detectionMode == Detection.BLUE_YELLOW)
            {
                double blue = 0.0, yellow = 0.0;
                if(maximumBlue != null)
                    blue = Imgproc.contourArea(maximumBlue);
                if(maximumYellow != null)
                    yellow = Imgproc.contourArea(maximumYellow);

                if(blue > yellow || (maximumYellow == null && maximumBlue != null)){
                    largestContour = maximumBlue;
                    contours = blues;
                }
                else if(blue < yellow || (maximumYellow != null && maximumBlue == null)){
                    largestContour = maximumYellow;
                    contours = yellows;
                }
            }
            else if(detectionMode == Detection.RED_YELLOW)
            {
                double red = 0.0, yellow = 0.0;
                if(maximumRed != null)
                    red = Imgproc.contourArea(maximumRed);
                if(maximumYellow != null)
                    yellow = Imgproc.contourArea(maximumYellow);

                if(red > yellow || (maximumYellow == null && maximumRed != null)){
                    largestContour = maximumRed;
                    contours = reds;
                }
                else if(red < yellow || (maximumYellow != null && maximumRed == null)){
                    largestContour = maximumYellow;
                    contours = yellows;
                }
            }

            if (largestContour != null) {
                Rect boundingRect = Imgproc.boundingRect(largestContour);

                int x = boundingRect.x;
                int y = boundingRect.y;
                int w = boundingRect.width;
                int h = boundingRect.height;

                cX = x + w / 2.0;
                cY = y + h / 2.0;

                tx = (cX - input.width() / 2.0) / (input.width() / 2.0);
                ty = (cY - input.height() / 2.0) / (input.height() / 2.0);


                Imgproc.drawContours(input, contours, contours.indexOf(largestContour), new Scalar(0, 255, 255), 2);
                width = calculateWidth(largestContour);

                RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(largestContour.toArray()));

                angle = getAngle(rect);

                Imgproc.putText(input, "Tx: " + String.format("%.3f", getTx()) + " m, Ty: " + String.format("%.3f", getTy()) + " m", new Point(cX + 5, cY + 80), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                Imgproc.putText(input, "Distance: " + String.format("%.2f", getDistance(width)) + " in", new Point(cX + 5, cY + 40), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                Imgproc.putText(input, "Angle: " + String.format("%.2f", angle) + " deg", new Point(cX + 5, cY + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);
            }

            return input;
        }

        private Mat preprocessBlue(Mat frame) {
            Mat hsvFrame = new Mat();
            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

            Scalar lowerBlue = new Scalar(0, 70, 199);
            Scalar upperBlue = new Scalar(12, 216, 255);

            //0 100 100
            //50 255 255

            Mat BlueMask = new Mat();
            Core.inRange(hsvFrame, lowerBlue, upperBlue, BlueMask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(BlueMask, BlueMask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(BlueMask, BlueMask, Imgproc.MORPH_CLOSE, kernel);

            return BlueMask;
        }

        private Mat preprocessRed(Mat frame) {
            Mat hsvFrame = new Mat();
            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

            // Good range for yellow
            Scalar lowerRed = new Scalar(115, 88, 0);
            Scalar upperRed = new Scalar(154, 255, 255);

            Mat redMask = new Mat();
            Core.inRange(hsvFrame, lowerRed, upperRed, redMask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(redMask, redMask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(redMask, redMask, Imgproc.MORPH_CLOSE, kernel);

            return redMask;
        }

        private Mat preprocessYellow(Mat frame) {
            Mat hsvFrame = new Mat();
            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_BGR2HSV);

            // Good range for yellow
            Scalar lowerYellow = new Scalar(17, 51, 149);
            Scalar upperYellow = new Scalar(121, 255, 255);


            Mat yellowMask = new Mat();
            Core.inRange(hsvFrame, lowerYellow, upperYellow, yellowMask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(yellowMask, yellowMask, Imgproc.MORPH_CLOSE, kernel);

            return yellowMask;
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
    private double calculateHeight(MatOfPoint contour) {
        Rect boundingRect = Imgproc.boundingRect(contour);
        return boundingRect.height;
    }

    public static double getDistance(double width) {
        return (objectWidthInRealWorldUnits * 720) / width;
    }

    public double getTx(){
        return tx;
    }

    public double getTy(){
        return ty;
    }
    public double getAngle(RotatedRect rect) {
        if (rect == null || rect.boundingRect().width == 0)
            return Double.NaN;

        double w = rect.size.width;
        double h = rect.size.height;
        double a = rect.angle;

        if(w < h)
            a = a + 90;

        return a;
    }

}
