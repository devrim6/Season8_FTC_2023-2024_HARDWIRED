package org.firstinspires.ftc.teamcode.cameraStuff;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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
    private FtcDashboard dash = FtcDashboard.getInstance();

    public AprilTagProcessor april = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

    public void detectATags(Pose2d currentPose, TelemetryPacket packet){
        aprilTagDetections.clear();
        aprilTagDetections = april.getDetections();
        if(aprilTagDetections.size()!=0) getEstimatedPosition(currentPose, packet);
    }

    private void getEstimatedPosition(Pose2d currentPose, TelemetryPacket packet){
        double allX=0, allY=0, allHeading=0;
        Canvas fieldOverlay = packet.fieldOverlay();
        for(AprilTagDetection detection : aprilTagDetections){
            if(detection.metadata!=null){
                // 0 - x, 1 - y, 2 - z
                float[] tagPos = detection.metadata.fieldPosition.getData();
                double tagX = tagPos[0], tagY = tagPos[1], tagZ = tagPos[2];
                fieldOverlay.setStroke("#FF0000");
                fieldOverlay.strokeCircle(tagX, tagY, 2);

                allX += tagX + Math.cos(detection.ftcPose.bearing)/detection.ftcPose.range;
                allY += tagY + Math.sin(detection.ftcPose.bearing)/detection.ftcPose.range;
                // Only works with pixel stacks (if it works at all) todo: think of a better way
                allHeading += Math.toRadians(180 + detection.ftcPose.bearing);
            }
        }
        double size = aprilTagDetections.size();
        if(size!=0){
            // Uses the average of all poses calculated. Might be changed.
            estimatedPose =  new Pose2d(allX/size, allY/size, allHeading/size);
            fieldOverlay.setStroke("#008000");
            fieldOverlay.strokeCircle(allX/size, allY/size, 2);
            return;
        }
        estimatedPose = currentPose;
    }

    /**
     * Make sure to add a .detectATags call before calling this method.
     * @param telemetry
     * @param packet
     */
    public void aprilTagTelemetry(@NonNull Telemetry telemetry, TelemetryPacket packet){
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

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}
