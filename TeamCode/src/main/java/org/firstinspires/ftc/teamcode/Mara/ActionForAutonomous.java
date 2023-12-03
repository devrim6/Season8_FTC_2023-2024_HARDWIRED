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

    Action Traj1= drive.actionBuilder(firstPose)
            .setReversed(false)
            .splineToLinearHeading(new Pose2d(-37, -62, Math.toRadians(90)), Math.toRadians(90))
            .splineToLinearHeading(new Pose2d(-59,-36,Math.toRadians(0)),Math.toRadians(0))
            .build();
}
