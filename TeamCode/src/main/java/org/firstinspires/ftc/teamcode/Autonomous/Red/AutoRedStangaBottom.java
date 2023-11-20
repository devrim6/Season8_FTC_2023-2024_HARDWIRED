package org.firstinspires.ftc.teamcode.Autonomous.Red;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareMapping;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PoseTransfer;

@Autonomous(name = "AutoRedStangaBottom")
public class AutoRedStangaBottom extends LinearOpMode {
    /* Init whatever you need */
    HardwareMapping robot = new HardwareMapping();
    HardwareMapping.Intake intake = robot.new Intake();
    HardwareMapping.Outtake outtake = robot.new Outtake();
    enum traj {
        TRAJ1_MiddleLine,
        TRAJ2_MiddleLineToStack,
        TRAJ3_StackToMiddleBackboard,
        IDLE
    }

    String elementPosition = "middle";
    traj currentTraj = traj.TRAJ1_MiddleLine;
    public void runOpMode() throws InterruptedException{
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(58, -34.5, Math.toRadians(270)));
        robot.init(hardwareMap);
        robot.resetEncoder();

        Action StartToMiddleLine = drive.actionBuilder(drive.pose)
                .splineToLinearHeading(new Pose2d(-32.5, -33, Math.toRadians(80)), Math.toRadians(90))
                //displacement markers and all that stuff
                .build();

        Action MiddleLineToStack = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .splineTo(new Vector2d(-57, -36), Math.toRadians(180))
                .build();

        Action StackToMiddleBackboard = drive.actionBuilder(drive.pose)
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-34, -58.5, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-10, -58.5, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(25, -58.5, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(49, -36, Math.toRadians(0)), Math.toRadians(0))
                .afterDisp(10, new ParallelAction(
                        outtake.runToPosition(HardwareMapping.liftHeight.LOW),
                        outtake.yaw(90)
                ))
                .build();

        waitForStart();

        if(isStopRequested()) return;
        Actions.runBlocking(StartToMiddleLine);

        while (opModeIsActive() && !isStopRequested()){
            // Finite state
            switch (currentTraj){
                case TRAJ1_MiddleLine:
                    if(!drive.isBusy()){
                        currentTraj = traj.TRAJ2_MiddleLineToStack;
                        Actions.runBlocking(MiddleLineToStack);
                    }
                    break;
                case TRAJ2_MiddleLineToStack:
                    if(!drive.isBusy()){
                        currentTraj = traj.TRAJ3_StackToMiddleBackboard;
                        Actions.runBlocking(StackToMiddleBackboard);
                    }
                    break;
                case IDLE:
                    break;
            }

            /* Transfer last pose to TeleOP */
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
