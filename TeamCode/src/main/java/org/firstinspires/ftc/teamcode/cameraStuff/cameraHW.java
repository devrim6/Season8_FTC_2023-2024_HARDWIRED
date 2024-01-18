package org.firstinspires.ftc.teamcode.cameraStuff;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class cameraHW {
    public static Rect rect1 = new Rect(0, 230, 70, 70);
    public static Rect rect2 = new Rect(290, 160, 70, 70);
    public static Rect rect3 = new Rect(565, 230, 70, 70);


    int ok;

    public Scalar lowerBlue =new Scalar(106, 100, 50);
    public Scalar upperBlue =new Scalar (230, 255, 255);


    public Scalar lowerRed = new Scalar(100, 100, 100);
    public Scalar uppeerRed = new Scalar(100, 100, 100);

    OpenCvCamera externalCamera;
    static ZoneDetector pipeline;


    public void initTeamPropCamera(String AllianceColor) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()
        );

        externalCamera = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "WebcamG"), cameraMonitorViewId
        );


        if(AllianceColor == "RED")
            ok=1;
        else if(AllianceColor == "BLUE")
            ok=0;


        pipeline = new ZoneDetector();
        externalCamera.setPipeline(pipeline);

        externalCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                externalCamera.startStreaming(640, 480, OpenCvCameraRotation.UPSIDE_DOWN);
            }
            @Override
            public void onError(int errorCode) {

            }
        });
    }
   public  String isPointInsideRect() {
        Point point = pipeline.getSquareCenter();
        if (rect1.contains(point)) {
            return "Left";

        } else if (rect2.contains(point)) {
            return "Middle";

        } else if (rect3.contains(point)) {
            return "Right";
        }
        return "Middle";
    }


    public class ZoneDetector extends OpenCvPipeline
    {
        private Mat hsvImage = new Mat();
        private Mat mask = new Mat();
        private Mat hierarchy = new Mat();

        private List<MatOfPoint> contours = new ArrayList<>();
        private Point squareCenter = new Point();
        public Scalar nonSelectedColor = new Scalar(255, 0, 0);
        public Scalar selectedColor = new Scalar(0, 0, 255);

        private int selectedRect = -1;

        @Override
        public Mat processFrame(Mat input)
        {

            Imgproc.cvtColor(input, hsvImage, Imgproc.COLOR_RGB2HSV);


            if(ok == 0)
                Core.inRange(hsvImage, lowerBlue, upperBlue, mask);
            else if(ok == 1)
                Core.inRange(hsvImage, lowerRed, uppeerRed, mask);


            contours.clear();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            Mat processedFrame = input.clone();
            Imgproc.drawContours(processedFrame, contours, -1, new Scalar(0, 255, 0), 2);

            // Calculate the position of the detected square
            calculateSquarePosition();

            // Draw coordinates on the processed frame
            if (!contours.isEmpty())
            {
                String coordinates = "X: " + String.format("%.2f", squareCenter.x) + " Y: " + String.format("%.2f", squareCenter.y);
                Imgproc.putText(
                        processedFrame,
                        coordinates,
                        new Point(10, 130),
                        Imgproc.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        new Scalar(0, 0, 255),
                        1
                );
            }

            drawRectangles(processedFrame);

            return processedFrame;
        }
        public Point getSquareCenter() {
            return squareCenter;
        }


        private void drawRectangles(Mat input)
        {
            Imgproc.rectangle(input, rect1, nonSelectedColor);
            Imgproc.rectangle(input, rect2, nonSelectedColor);
            Imgproc.rectangle(input, rect3, nonSelectedColor);

            switch (selectedRect)
            {
                case 1:
                    Imgproc.rectangle(input, rect1, selectedColor);
                    break;
                case 2:
                    Imgproc.rectangle(input, rect2, selectedColor);
                    break;
                case 3:
                    Imgproc.rectangle(input, rect3, selectedColor);
                    break;
            }
        }

        private void calculateSquarePosition()
        {
            squareCenter.x = 0;
            squareCenter.y = 0;

            if (!contours.isEmpty())
            {
                double maxArea = -1;
                int maxAreaIdx = -1;
                for (int i = 0; i < contours.size(); i++)
                {
                    double area = Imgproc.contourArea(contours.get(i));
                    if (area > maxArea)
                    {
                        maxArea = area;
                        maxAreaIdx = i;
                    }
                }

                if (maxAreaIdx != -1)
                {
                    // Calculate the center of the largest contour
                    Moments moments = Imgproc.moments(contours.get(maxAreaIdx));
                    squareCenter.x = moments.m10 / moments.m00;
                    squareCenter.y = moments.m01 / moments.m00;
                }
            }
        }
    }
}
