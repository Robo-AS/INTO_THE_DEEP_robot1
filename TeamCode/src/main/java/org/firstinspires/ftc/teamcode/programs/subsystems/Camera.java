package org.firstinspires.ftc.teamcode.programs.subsystems;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

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

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class Camera extends OpMode {
    OpenCvWebcam webcam = null;

    @Override
    public void init(){
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam1");
        int cameraID = hardwareMap.appContext.getResources().getIdentifier("cameraID", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraID);

        webcam.setPipeline(new colorDetection());

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("camera error: ", errorCode);
                telemetry.update();
            }
        });
    }

    @Override
    public void loop(){
        telemetry.addData("camera status: ", "streaming");
        telemetry.update();
    }

    public class colorDetection extends OpenCvPipeline{
        private Mat hsvMat = new Mat();
        private Mat mask = new Mat();
        private Mat hierarchy = new Mat();
        private Scalar lowerBound = new Scalar(100, 150, 70);
        private Scalar upperBound = new Scalar(125, 255, 255);

        @Override
        public Mat processFrame(Mat input){
            Imgproc.cvtColor(input, hsvMat, Imgproc.COLOR_RGB2HSV);
            Core.inRange(hsvMat, lowerBound, upperBound, mask);

            List<MatOfPoint> contours = new ArrayList<>();

            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            if(!contours.isEmpty()) {
                MatOfPoint largestContour = contours.get(0);
                double maxArea = 0;

                for(MatOfPoint contour : contours)
                {
                    double area = Imgproc.contourArea(contour);
                    if(area > maxArea){
                        maxArea = area;
                        largestContour = contour;
                    }
                }

                Rect boundingRect = Imgproc.boundingRect(largestContour);
                Imgproc.rectangle(input, boundingRect, new Scalar(0, 255, 0), 2);

                Point center = new Point(boundingRect.x + boundingRect.width / 2.0, boundingRect.y + boundingRect.height / 2.0);
                Imgproc.circle(input, center, 5, new Scalar(0, 255, 0), -1);
                Imgproc.putText(input, "target colour found", new Point(center.x - 20, center.y - 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
            }

            return input;
        }

    }

}
