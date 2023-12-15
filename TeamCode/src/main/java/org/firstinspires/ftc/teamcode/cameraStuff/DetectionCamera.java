package org.firstinspires.ftc.teamcode.cameraStuff;

import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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
        TEAM_ELEMENT,
        PIXEL
    }

    private AprilTagPipeline april;

    private VisionPortal.Builder builder;
    private TfodProcessor pixel;
    private TeamElementPipeline teamElement;
    private VisionPortal visionPortal;

    /**
     * Initialize the camera, set up Vision Portal + the ATag processor and set the required processor
     * @param hwMap
     * @param proc
     */
    public void initCamera(@NonNull HardwareMap hwMap, processor proc){
        builder = new VisionPortal.Builder();
        builder.setCamera(hwMap.get(WebcamName.class, "Webcam 1"));

        builder.setCameraResolution(new Size(640, 480));
        builder.enableLiveView(true);
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        pixel = new TfodProcessor.Builder()
                .build();

        //todo: add teamElement after it's done
        builder.addProcessors(april.april, pixel);

        enableProcessor(proc);

        visionPortal = builder.build();
    }

    public void enableStreaming(boolean a){
        if(a) visionPortal.resumeStreaming();
        else visionPortal.stopStreaming();
    }

    /**
     * Enables a processor and disables the other ones.
     * @param proc
     */
    public void enableProcessor(@NonNull processor proc){
        switch (proc){
            case APRIL_TAG:
                visionPortal.setProcessorEnabled(april.april,true);
                visionPortal.setProcessorEnabled(pixel,false);
                //visionPortal.setProcessorEnabled(teamElement, false);
                break;
            case PIXEL:
                visionPortal.setProcessorEnabled(april.april,false);
                visionPortal.setProcessorEnabled(pixel,true);
                //visionPortal.setProcessorEnabled(teamElement, false);
                break;
            case TEAM_ELEMENT:
                visionPortal.setProcessorEnabled(april.april,false);
                visionPortal.setProcessorEnabled(pixel,false);
                //visionPortal.setProcessorEnabled(teamElement, true);
                break;
        }
    }

    public void stopCamera(){visionPortal.close();}

    public void aprilTagTelemetry(Telemetry telemetry, TelemetryPacket packet, Pose2d currentPose)
    {april.aprilTagTelemetry(telemetry, packet, currentPose);}

    public Pose2d getPoseEstimate(){return april.estimatedPose;}

}
