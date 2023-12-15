package org.firstinspires.ftc.teamcode.cameraStuff;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AprilTagPipeline {

    private AprilTagDetection close;
    private List<AprilTagDetection> aprilTagDetections;
    public Pose2d estimatedPose;

    public AprilTagProcessor april = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

    private void detectATags(Pose2d currentPose){
        aprilTagDetections.clear();
        aprilTagDetections = april.getDetections();
        if(aprilTagDetections.size()!=0) getEstimatedPosition(currentPose);
    }

    private void getEstimatedPosition(Pose2d currentPose){
        double allX=0, allY=0, allHeading=0;
        for(AprilTagDetection detection : aprilTagDetections){
            if(detection.metadata!=null){
                // 0 - x, 1 - y, 2 - z
                float[] tagPos = detection.metadata.fieldPosition.getData();
                double tagX = tagPos[0], tagY = tagPos[1], tagZ = tagPos[2];

                allX+=tagX + Math.cos(detection.ftcPose.bearing)/detection.ftcPose.range;
                allY+=tagY + Math.sin(detection.ftcPose.bearing)/detection.ftcPose.range;
                // Only works with pixel stacks (if it works at all) todo: think of a better way
                allHeading+=Math.toRadians(180 + detection.ftcPose.bearing);
            }
        }
        double size = aprilTagDetections.size();
        if(size!=0){
            // Uses the average of all poses calculated. Might be changed.
            estimatedPose =  new Pose2d(allX/size, allY/size, allHeading/size);
            return;
        }
        estimatedPose = currentPose;
    }

    public void aprilTagTelemetry(@NonNull Telemetry telemetry, Pose2d currentPose){
        detectATags(currentPose);
        telemetry.addData("Nr of April Tags detected: ", aprilTagDetections.size());
        double minDis = 999; // It's never gonna be 999 inches lol

        for(AprilTagDetection detection : aprilTagDetections){
            if(detection.metadata!=null){
                telemetry.addLine("\n");
                telemetry.addLine("ID detected: " + detection.id);
                telemetry.addLine("ATag name: " + detection.metadata.name);
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
        telemetry.addLine("Estimated pose x: " + estimatedPose.position.x + "y: " + estimatedPose.position.y);

    }


}
