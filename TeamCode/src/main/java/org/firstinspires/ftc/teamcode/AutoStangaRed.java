package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class AutoStangaRed extends LinearOpMode {
    /* Init whatever you need */
    HardwareMapping robot = new HardwareMapping();
    HardwareMapping.Intake intake = robot.new Intake();
    HardwareMapping.Outtake outtake = robot.new Outtake();
    enum state {
        TRAJ1,
        TRAJ2,
        IDLE
    }

    String elementPosition = "middle";
    state currentState = state.TRAJ1;
    public void runOpMode() throws InterruptedException{
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(58, -34.5, Math.toRadians(270)));
        robot.init(hardwareMap);
        robot.resetEncoder();

        Action StartToMiddle = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(34.5, -34.5), Math.PI/2)
                .splineTo(new Vector2d(34.5, -57.5), Math.PI/2)
                .build();

        waitForStart();

        if(isStopRequested()) return;
        Actions.runBlocking(StartToMiddle);

        while (opModeIsActive() && !isStopRequested()){
            // Finite state
            switch (currentState){
                case TRAJ1:
                case IDLE:
                    break;
            }

            /** Transfer last pose to TeleOP */
            PoseTransfer.currentPose = drive.pose;
            PoseTransfer.glisieraTicks1 = robot.slideMotorRight.getCurrentPosition();
            PoseTransfer.glisieraTicks2 = robot.slideMotorLeft.getCurrentPosition();

            drive.updatePoseEstimate();

            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading", drive.pose.heading);
        }
    }
}
