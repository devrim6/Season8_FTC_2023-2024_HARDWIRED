package org.firstinspires.ftc.teamcode.Mara;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.HardwareMapping;
import org.firstinspires.ftc.teamcode.MecanumDrive;

public class ActionForAutonomous{
    //HardwareMap hw=null;
    HardwareMapp robot = new HardwareMapp();
    MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(58, -34.5, Math.toRadians(270)));

    Pose2d firstPose=new Pose2d(-37,-34,90);
    Pose2d backboardPose=new Pose2d(48,-34,0);
    Pose2d stackPose=new Pose2d(-55,-11,0);

    Action Autonomie1= drive.actionBuilder(firstPose) //Autonomie1

            .setReversed(false)
            .splineToLinearHeading(new Pose2d(-29, -33, Math.toRadians(60)), Math.toRadians(60))
            .waitSeconds(0.2)
            .setReversed(true)
            .splineToLinearHeading(new Pose2d(-53,-36,Math.toRadians(0)),Math.toRadians(90))
            .waitSeconds(0.7) //ia un pixel
            //1
            .setTangent(Math.toRadians(90))
            .splineToLinearHeading(new Pose2d(-38.39, -12.02,Math.toRadians(0)), Math.toRadians(0))
            .splineToLinearHeading(new Pose2d(27, -12.02,Math.toRadians(0)), Math.toRadians(0))
            .splineToLinearHeading(new Pose2d(48, -41.31,Math.toRadians(0)), Math.toRadians(0))
            .waitSeconds(0.7)
            .setReversed(true)
            .splineToLinearHeading(new Pose2d(27, -12.02,Math.toRadians(0)), Math.toRadians(180))
            .splineToLinearHeading(new Pose2d(-56.45, -12.02,Math.toRadians(0)), Math.toRadians(180))
            .setReversed(false)
            .waitSeconds(0.7)
            .splineToLinearHeading(new Pose2d(27, -12.02,Math.toRadians(0)), Math.toRadians(0))
            .splineToLinearHeading(new Pose2d(48, -29.65,Math.toRadians(0)), Math.toRadians(0))
            .waitSeconds(0.7)
            .setReversed(true)
            .splineToLinearHeading(new Pose2d(27, -12.02,Math.toRadians(0)), Math.toRadians(180))
            .splineToLinearHeading(new Pose2d(-56.45, -12.02,Math.toRadians(0)), Math.toRadians(180))
            .setReversed(false)
            .waitSeconds(0.7)
            .splineToLinearHeading(new Pose2d(27, -12.02,Math.toRadians(0)), Math.toRadians(0))
            .splineToLinearHeading(new Pose2d(48, -29.65,Math.toRadians(0)), Math.toRadians(0))
            .waitSeconds(0.7)
            //de final
            .setReversed(true)
            .setTangent(Math.toRadians(90))
            .splineToSplineHeading(new Pose2d(48,-15,Math.toRadians(0)),Math.toRadians(90))
            .build();

    Action Autonomie2= drive.actionBuilder(firstPose) //Autonomie2

            .setReversed(false)
            .splineToLinearHeading(new Pose2d(-29, -33, Math.toRadians(60)), Math.toRadians(60))
            .waitSeconds(0.2)
            .setReversed(true)
            .splineToLinearHeading(new Pose2d(-53,-36,Math.toRadians(0)),Math.toRadians(90))
            .waitSeconds(0.7) //ia un pixel
            //1
            .setTangent(Math.toRadians(90))
            .splineToLinearHeading(new Pose2d(-38.39, -12.02,Math.toRadians(0)), Math.toRadians(0))
            .splineToLinearHeading(new Pose2d(27, -12.02,Math.toRadians(0)), Math.toRadians(0))
            .splineToLinearHeading(new Pose2d(48, -41.31,Math.toRadians(0)), Math.toRadians(0))
            .waitSeconds(0.7)
            .setReversed(true)
            .splineToLinearHeading(new Pose2d(27, -12.02,Math.toRadians(0)), Math.toRadians(180))
            .splineToLinearHeading(new Pose2d(-56.45, -12.02,Math.toRadians(0)), Math.toRadians(180))
            .setReversed(false)
            .waitSeconds(0.7)
            .splineToLinearHeading(new Pose2d(27, -12.02,Math.toRadians(0)), Math.toRadians(0))
            .splineToLinearHeading(new Pose2d(48, -29.65,Math.toRadians(0)), Math.toRadians(0))
            .waitSeconds(0.7)
            .setReversed(true)
            .splineToLinearHeading(new Pose2d(27, -12.02,Math.toRadians(0)), Math.toRadians(180))
            .splineToLinearHeading(new Pose2d(-56.45, -12.02,Math.toRadians(0)), Math.toRadians(180))
            .setReversed(false)
            .waitSeconds(0.7)
            .splineToLinearHeading(new Pose2d(27, -12.02,Math.toRadians(0)), Math.toRadians(0))
            .splineToLinearHeading(new Pose2d(48, -29.65,Math.toRadians(0)), Math.toRadians(0))
            .waitSeconds(0.7)
            //de final
            .setReversed(true)
            .setTangent(Math.toRadians(90))
            .splineToLinearHeading(new Pose2d(48,-15,Math.toRadians(0)),Math.toRadians(90))
            .build();
}