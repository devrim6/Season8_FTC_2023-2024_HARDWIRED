package org.firstinspires.ftc.teamcode.cameraStuff;

import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.ArrayList;
import java.util.List;

public class DetectionCamera {
    public DetectionCamera(){}
    public enum processor {
        APRIL_TAG,
        PIXEL
    }

    private VisionPortal.Builder builder;
    private AprilTagProcessor april;
    private TfodProcessor pixel;
    private VisionPortal visionPortal;
    private AprilTagDetection close;
    private List<AprilTagDetection> aprilTagDetections;

    public void initCamera(@NonNull HardwareMap hwMap, processor proc){
        builder = new VisionPortal.Builder();
        builder.setCamera(hwMap.get(WebcamName.class, "Webcam 1"));

        builder.setCameraResolution(new Size(640, 480));
        builder.enableLiveView(true);
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        setProcessor(proc);

        visionPortal = builder.build();
    }

    public void enableStreaming(boolean a){
        if(a) visionPortal.resumeStreaming();
        else visionPortal.stopStreaming();
    }
    public void stopCamera(){visionPortal.close();}

    public void setProcessor(@NonNull processor proc){
        switch (proc){
            case APRIL_TAG:
                april = new AprilTagProcessor.Builder()
                        .setDrawAxes(false)
                        .setDrawCubeProjection(false)
                        .setDrawTagOutline(true)
                        .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                        .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                        .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                        .build();
                builder.addProcessor(april);
                break;
            case PIXEL:
                //todo: make a pixel stack detection algorithm
                //builder.addProcessor();
                break;
        }
    }

    public void detectTags(){
        aprilTagDetections.clear();
        aprilTagDetections = april.getDetections();
    }

    public Pose2d getEstimatedPosition(Pose2d currentPose){
        double allX=0, allY=0, allHeading=0;
        for(AprilTagDetection detection : aprilTagDetections){
            if(detection.metadata!=null){
                // 0 - x, 1 - y, 2 - z
                float[] tagPos = detection.metadata.fieldPosition.getData();
                double tagX = tagPos[0], tagY = tagPos[1], tagZ = tagPos[2];

                allX+=tagX + Math.sin(detection.ftcPose.bearing)/detection.ftcPose.range;
                allY+=tagY + Math.cos(detection.ftcPose.bearing)/detection.ftcPose.range;
                // Only works with pixel stacks (if it works at all) todo: think of a better way
                allHeading+=Math.toRadians(180 + detection.ftcPose.bearing);
            }
        }
        double size = aprilTagDetections.size();
        if(size!=0){
            // Uses the average of all poses calculated. Might be changed.
            return new Pose2d(allX/size, allY/size, allHeading/size);
        }
        return currentPose;
    }

    public void aprilTagTelemetry(@NonNull Telemetry telemetry){
        telemetry.addData("Nr of April Tags detected: ", aprilTagDetections.size());
        double minDis = 999; // It's never gonna be 999 inches lol

        for(AprilTagDetection detection : aprilTagDetections){
            if(detection.metadata!=null){
                telemetry.addLine("\n");
                telemetry.addLine("ID detected: " + detection.id);
                telemetry.addLine("Distance from camera: " + detection.ftcPose.range);
                telemetry.addLine("Angle from camera: " + detection.ftcPose.bearing);
                telemetry.addLine("Elevation from camera: " + detection.ftcPose.elevation);
                if(detection.ftcPose.range <= minDis) minDis = detection.ftcPose.range;
                close = detection;
            } else {
                telemetry.addLine("\n");
                telemetry.addLine("Unknown ID detected: " + detection.id);
                telemetry.addLine("Distance from camera: " + detection.ftcPose.range);
                telemetry.addLine("Angle from camera: " + detection.ftcPose.bearing);
            }
        }

        telemetry.addLine("Last seen AprilTag: " + close.id);

    }

}
