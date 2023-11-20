package org.firstinspires.ftc.teamcode.Autonomous.Red;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
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
        TRAJ4_MiddleBackboardToStack,
        TRAJ5_StackToRightBackboard,
        TRAJ6_RightBackboardToStack,
        TRAJ7_ParkRight,
        IDLE
    }

    boolean isIntakePowered=false, areHooksEngaged=true, intakeManualControl=true;
    HardwareMapping.ledState bottomSensorState = HardwareMapping.ledState.OFF, upperSensorState = HardwareMapping.ledState.OFF;
    long currentTime = System.currentTimeMillis();

    String elementPosition = "middle";
    traj currentTraj = traj.TRAJ1_MiddleLine;
    public void runOpMode() throws InterruptedException{
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(58, -34.5, Math.toRadians(270)));
        robot.init(hardwareMap);
        robot.resetEncoder();

        Action TRAJ1_MiddleLine = drive.actionBuilder(drive.pose)
                .splineToLinearHeading(new Pose2d(-32.5, -33, Math.toRadians(80)), Math.toRadians(90))
                //displacement markers and all that stuff
                .build();

        Action TRAJ2_MiddleLineToStack = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .splineTo(new Vector2d(-57, -36), Math.toRadians(180))
                .afterDisp(1, intake.powerOn())
                .build();

        Action TRAJ3_StackToMiddleBackboard = drive.actionBuilder(drive.pose)
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-34, -58.5, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-10, -58.5, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(25, -58.5, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(49, -36, Math.toRadians(0)), Math.toRadians(0))
                .afterDisp(10, new SequentialAction(
                        outtake.runToPosition(HardwareMapping.liftHeight.LOW),
                        outtake.yaw(90)
                ))
                .build();

        Action TRAJ4_MiddleBackboardToStack = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(25, -58.5, Math.toRadians(0)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-10, -58.5, Math.toRadians(0)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-34, -58.5, Math.toRadians(0)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-57, -36, Math.toRadians(0)), Math.toRadians(180))
                .build();

        Action TRAJ5_StackToRightBackboard = drive.actionBuilder(drive.pose)
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-34, -58.5, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-10, -58.5, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(25, -58.5, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(49, -42, Math.toRadians(0)), Math.toRadians(0))
                .afterDisp(10, new SequentialAction(
                        outtake.runToPosition(HardwareMapping.liftHeight.LOW),
                        outtake.yaw(90)
                ))
                .build();

        Action TRAJ6_RightBackboardToStack = drive.actionBuilder(drive.pose)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(25, -58.5, Math.toRadians(0)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-10, -58.5, Math.toRadians(0)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-34, -58.5, Math.toRadians(0)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-57, -36, Math.toRadians(0)), Math.toRadians(180))
                .build();

        Action TRAJ7_ParkRight = drive.actionBuilder(drive.pose)
                .setReversed(false)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(49, -55, Math.toRadians(0)), Math.toRadians(-90))
                .afterDisp(0.1, new SequentialAction(
                        new ParallelAction(
                                outtake.yaw(0),
                                outtake.bottomHook("open"),
                                outtake.upperHook("open")
                        ),
                        outtake.runToPosition(HardwareMapping.liftHeight.LOW)
                ))
                .splineToLinearHeading(new Pose2d(57.5, -60, Math.toRadians(0)), Math.toRadians(0))
                .build();

        waitForStart();

        if(isStopRequested()) return;
        Actions.runBlocking(TRAJ1_MiddleLine);

        while (opModeIsActive() && !isStopRequested()){
            // Finite state
            switch (currentTraj){
                case TRAJ1_MiddleLine:
                    if(!drive.isBusy()){
                        currentTraj = traj.TRAJ2_MiddleLineToStack;
                        Actions.runBlocking(TRAJ2_MiddleLineToStack);
                        isIntakePowered = true;
                    }
                    break;
                case TRAJ2_MiddleLineToStack:
                    if(!drive.isBusy()){
                        currentTraj = traj.TRAJ3_StackToMiddleBackboard;
                        Actions.runBlocking(TRAJ3_StackToMiddleBackboard);
                    }
                    break;
                case IDLE:
                    break;
            }

            //If intake is running and two pixels are already in then reverse the intake and lower the hooks
            //TODO: implement independent closing in case if one hook is engaged and the other is not, priority is the closed hook
            if(isIntakePowered){
                upperSensorState = robot.checkColorRange("upper");
                bottomSensorState = robot.checkColorRange("bottom");
                if(!upperSensorState.equals(HardwareMapping.ledState.OFF) && !bottomSensorState.equals(HardwareMapping.ledState.OFF)) {
                    if(System.currentTimeMillis()> currentTime + 500){ //Timer so that the bot is sure there are two pixels inside and doesn't have false positives
                        Actions.runBlocking(new ParallelAction(
                                outtake.bottomHook("closed"),
                                outtake.upperHook("closed"),
                                intake.reverse()
                        ));                                                     // Reverse intake to filter out potential third pixel
                        areHooksEngaged = true;                                 // todo: implement beam break
                        isIntakePowered = false;
                        intakeManualControl = false;
                    }
                } else currentTime = System.currentTimeMillis();
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
