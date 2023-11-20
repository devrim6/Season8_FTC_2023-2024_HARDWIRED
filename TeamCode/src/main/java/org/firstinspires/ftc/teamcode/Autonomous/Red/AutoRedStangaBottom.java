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
        TRAJ1_StartToLine,
        TRAJ2_MiddleLineToStack,

        TRAJ3_StackToMiddleBackboard,
        TRAJ4_MiddleBackboardToStack,

        TRAJ5_StackToRightBackboard,
        TRAJ6_RightBackboardToStack,

        TRAJ7_ParkRight,
        IDLE
    }

    Pose2d stackPose = new Pose2d(-57, -36, Math.toRadians(180)),
           middleBackboardPose = new Pose2d(49, -36, Math.toRadians(0)),
           rightBackboardPose = new Pose2d(49, -42, Math.toRadians(0));

    boolean isIntakePowered=false, areHooksEngaged=true, intakeManualControl=true;
    HardwareMapping.ledState bottomSensorState = HardwareMapping.ledState.OFF, upperSensorState = HardwareMapping.ledState.OFF;
    long currentTime = System.currentTimeMillis();
    double traj5Counter = 0;

    String elementPosition = "middle";
    traj currentTraj = traj.TRAJ1_StartToLine;
    public void runOpMode() throws InterruptedException{
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(58, -34.5, Math.toRadians(270)));
        robot.init(hardwareMap);
        robot.resetEncoder();

        Action TRAJ1_MiddleLine = drive.actionBuilder(drive.pose)
                .splineToLinearHeading(new Pose2d(-32.5, -33, Math.toRadians(80)), Math.toRadians(90))
                //displacement markers and all that stuff
                .build();
        Action TRAJ1_LeftLine = drive.actionBuilder(drive.pose)
                .splineToLinearHeading(new Pose2d(-43, -33, Math.toRadians(110)), Math.toRadians(120))
                .build();
        Action TRAJ1_RightLine = drive.actionBuilder(drive.pose)
                .splineToLinearHeading(new Pose2d(-30, -33, Math.toRadians(60)), Math.toRadians(60))
                .build();

        Action TRAJ3_StackToMiddleBackboard = drive.actionBuilder(stackPose)
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-34, -58.5, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-10, -58.5, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(25, -58.5, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(middleBackboardPose, Math.toRadians(0))
                .afterDisp(10, new SequentialAction(
                        outtake.runToPosition(HardwareMapping.liftHeight.LOW),
                        outtake.yaw(90)
                ))
                .build();

        Action TRAJ4_MiddleBackboardToStack = drive.actionBuilder(middleBackboardPose)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(25, -58.5, Math.toRadians(0)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-10, -58.5, Math.toRadians(0)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-34, -58.5, Math.toRadians(0)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(stackPose.position, Math.toRadians(0)), Math.toRadians(180))
                .afterDisp(3, new ParallelAction(
                        intake.powerOn(),
                        intake.angle(3)     // Take two pixels. Remaining after: 2
                ))
                .build();

        Action TRAJ5_StackToRightBackboard = drive.actionBuilder(stackPose)
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-34, -58.5, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-10, -58.5, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(25, -58.5, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(rightBackboardPose, Math.toRadians(0))
                .afterDisp(10, new SequentialAction(
                        outtake.runToPosition(HardwareMapping.liftHeight.LOW),
                        outtake.yaw(90)
                ))
                .build();

        Action TRAJ6_RightBackboardToStack = drive.actionBuilder(rightBackboardPose)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(25, -58.5, Math.toRadians(0)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-10, -58.5, Math.toRadians(0)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-34, -58.5, Math.toRadians(0)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(stackPose.position, Math.toRadians(0)), Math.toRadians(180))
                .afterDisp(3, new ParallelAction(
                        intake.powerOn(),
                        intake.angle(1)     // Take two pixels. Remaining after: 0
                ))
                .build();

        Action TRAJ7_ParkRight = drive.actionBuilder(rightBackboardPose)
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

        Pose2d nextPose;

        waitForStart();

        if(isStopRequested()) return;

        switch (elementPosition) {
            case "left":
                Actions.runBlocking(TRAJ1_LeftLine);
                nextPose = new Pose2d(-43, -33, Math.toRadians(110));
                break;
            case "right":
                Actions.runBlocking(TRAJ1_RightLine);
                nextPose = new Pose2d(-30, -33, Math.toRadians(60));
                break;
            default:
                Actions.runBlocking(TRAJ1_MiddleLine);
                nextPose = new Pose2d(-32.5, -33, Math.toRadians(80));
                break;
        }

        Action TRAJ2_MiddleLineToStack = drive.actionBuilder(nextPose)
                .setReversed(true)
                .splineTo(stackPose.position, Math.toRadians(180))
                .afterDisp(1, new ParallelAction(
                        intake.powerOn(),
                        intake.angle(5)     // Take one pixel. Remaining after: 4
                ))
                .build();

        while (opModeIsActive() && !isStopRequested()){
            // Finite state
            switch (currentTraj){
                case TRAJ1_StartToLine:
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
                case TRAJ3_StackToMiddleBackboard:
                    if(!drive.isBusy()){
                        currentTraj = traj.TRAJ4_MiddleBackboardToStack;
                        isIntakePowered=true;
                        Actions.runBlocking(new ParallelAction(
                                TRAJ4_MiddleBackboardToStack,       // Just in case, stop the intake
                                intake.stop()                       // then start it again in the trajectory
                        ));
                    }
                    break;
                case TRAJ4_MiddleBackboardToStack:
                    if(!drive.isBusy()){
                        currentTraj = traj.TRAJ5_StackToRightBackboard;
                        Actions.runBlocking(TRAJ5_StackToRightBackboard);
                    }
                    break;
                case TRAJ5_StackToRightBackboard:
                    if(!drive.isBusy()){
                        isIntakePowered=true;
                        traj5Counter++;
                        if(traj5Counter == 1) {                        // If on the first cycle of right
                            currentTraj = traj.TRAJ6_RightBackboardToStack;
                            Actions.runBlocking(
                                    TRAJ6_RightBackboardToStack,       // Just in case, stop the intake
                                    intake.stop()                      // then start it again in the trajectory
                            );
                        }
                        else {                                         // If on the 2nd cycle of right
                            currentTraj = traj.TRAJ7_ParkRight;
                            Actions.runBlocking(
                                    TRAJ7_ParkRight,                   // Just in case, stop the intake
                                    intake.stop()                      // then start it again in the trajectory
                            );
                        }
                    }
                    break;
                case TRAJ6_RightBackboardToStack:
                    if(!drive.isBusy()){
                        currentTraj = traj.TRAJ7_ParkRight;
                        Actions.runBlocking(TRAJ7_ParkRight);
                    }
                    break;
                case TRAJ7_ParkRight:
                    if(!drive.isBusy()){
                        isIntakePowered = false;
                        currentTraj = traj.IDLE;
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
