package org.firstinspires.ftc.teamcode.cameraStuff;

import android.util.Size;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

public class DetectionCamera {
    public DetectionCamera(){}
    public enum processor {
        APRIL_TAG,
        PIXEL
    }
    public VisionPortal.Builder builder;
    public AprilTagProcessor april;
    public TfodProcessor pixel;
    private VisionPortal visionPortal;
    public AprilTagDetection close;
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

    public void aprilTagTelemetry(@NonNull Telemetry telemetry){
        List<AprilTagDetection> aprilTagDetections = april.getDetections();
        telemetry.addData("Nr of April Tags detected: ", aprilTagDetections.size());
        double minDis = 999;

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

        telemetry.addLine("Closest AprilTag: " + close.id);

    }

}
