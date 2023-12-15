package org.firstinspires.ftc.teamcode.Autonomous.Red;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.HardwareMapping;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PoseTransfer;
import org.firstinspires.ftc.teamcode.Variables.DefVal;

@Autonomous(group = "Auto", name = "AutoRedStangaBottom")
public class AutoRedStangaBottom extends LinearOpMode {
    /* Init whatever you need */
    HardwareMapping robot = new HardwareMapping();
    HardwareMapping.Intake intake = robot.new Intake();
    HardwareMapping.Outtake outtake = robot.new Outtake();
    HardwareMapping.Auto auto = robot.new Auto();
    enum traj {
        TRAJ1_StartToLine("TRAJ1_StartToLine"),
        TRAJ2_MiddleLineToStack("TRAJ2_MiddleLineToStack"),

        TRAJ3_StackToMiddleBackboard("TRAJ3_StackToMiddleBackboard"),
        TRAJ3_StackToLeftBackboard("TRAJ3_StackToLeftBackboard"),
        TRAJ3_StackToRightBackboard("TRAJ3_StackToRightBackboard"),

        TRAJ4_MiddleBackboardToStack("TRAJ4_MiddleBackboardToStack"),

        TRAJ5_StackToLeftBackboard("TRAJ5_StackToLeftBackboard"),
        TRAJ6_LeftBackboardToStack("TRAJ6_LeftBackboardToStack"),

        TRAJ7_ParkRight("TRAJ7_ParkRight"),
        IDLE("IDLE");

        public final String trajName;

        traj(String trajName){
            this.trajName = trajName;
        }
    }

    Pose2d  stackMiddlePose = new Pose2d(-57, -25, Math.toRadians(180)),
            stackUpperPose = new Pose2d(-57, -12.5, Math.toRadians(0)),
            middleBackboardPose = new Pose2d(49, -36, Math.toRadians(0)),
            rightBackboardPose = new Pose2d(49, -42, Math.toRadians(0)),
            leftBackboardPose = new Pose2d(49, -30, Math.toRadians(0));

    double cycleCounter = 0;

    String elementPosition = "middle";
    traj currentTraj = traj.TRAJ1_StartToLine;

    public void runOpMode() throws InterruptedException{
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(58, -34.5, Math.toRadians(270)));
        robot.init(hardwareMap);
        robot.resetEncoderAuto();

        Action TRAJ1_MiddleLine = drive.actionBuilder(drive.pose)
                .splineToLinearHeading(new Pose2d(-32.5, -33, Math.toRadians(80)), Math.toRadians(80))
                //displacement markers and all that stuff
                .build();
        Action TRAJ1_LeftLine = drive.actionBuilder(drive.pose)
                .splineToLinearHeading(new Pose2d(-43, -33, Math.toRadians(110)), Math.toRadians(120))
                .build();
        Action TRAJ1_RightLine = drive.actionBuilder(drive.pose)
                .splineToLinearHeading(new Pose2d(-30, -33, Math.toRadians(60)), Math.toRadians(60))
                .build();

        Action TRAJ3_StackToMiddleBackboard = drive.actionBuilder(stackMiddlePose)
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-34, -12.5, Math.toRadians(0)), Math.toRadians(0))
                .afterDisp(3, intake.angle(6))              // Higher intake to not get pixels
                .splineToLinearHeading(new Pose2d(25, -12.5, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(49, -36, Math.toRadians(0)), Math.toRadians(0))
                .afterDisp(10, new ParallelAction(
                        new SequentialAction(
                                new ParallelAction(
                                        outtake.runToPosition(HardwareMapping.liftHeight.LOW),
                                        outtake.pivot(DefVal.pivot60),
                                        outtake.roll(DefVal.roll60)
                                ),
                                outtake.yaw(DefVal.yaw90),
                                outtake.latch("open")
                        ),
                        intake.sensingOff()
                ))
                .build();
        Action TRAJ3_StackToLeftBackboard = drive.actionBuilder(stackMiddlePose)
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-34, -12.5, Math.toRadians(0)), Math.toRadians(0))
                .afterDisp(3, intake.angle(6))              // Higher intake to not get pixels
                .splineToLinearHeading(new Pose2d(25, -12.5, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(49, -30, Math.toRadians(0)), Math.toRadians(0))
                .afterDisp(10, new ParallelAction(
                        new SequentialAction(
                                new ParallelAction(
                                        outtake.runToPosition(HardwareMapping.liftHeight.LOW),
                                        outtake.pivot(DefVal.pivot60),
                                        outtake.roll(DefVal.roll60)
                                ),
                                outtake.yaw(DefVal.yaw90),
                                outtake.latch("open")
                        ),
                        intake.sensingOff()
                ))
                .build();
        Action TRAJ3_StackToRightBackboard = drive.actionBuilder(stackMiddlePose)
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-34, -12.5, Math.toRadians(0)), Math.toRadians(0))
                .afterDisp(3, intake.angle(6))              // Higher intake to not get pixels
                .splineToLinearHeading(new Pose2d(25, -12.5, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(49, -42, Math.toRadians(0)), Math.toRadians(0))
                .afterDisp(10, new ParallelAction(
                        new SequentialAction(
                                new ParallelAction(
                                        outtake.runToPosition(HardwareMapping.liftHeight.LOW),
                                        outtake.pivot(DefVal.pivot60),
                                        outtake.roll(DefVal.roll60)
                                ),
                                outtake.yaw(DefVal.yaw90),
                                outtake.latch("open")
                        ),
                        intake.sensingOff()
                ))
                .build();

        Action TRAJ4_MiddleBackboardToStack = drive.actionBuilder(middleBackboardPose)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(25, -12.5, Math.toRadians(0)), Math.toRadians(180))
                .afterDisp(3, new SequentialAction(
                        new ParallelAction(
                                outtake.latch("closed"),
                                outtake.yaw(DefVal.yaw0)
                        ),
                        new ParallelAction(
                                outtake.pivot(DefVal.pivot0),
                                outtake.roll(DefVal.roll0)
                        ),
                        outtake.runToPosition(HardwareMapping.liftHeight.GROUND)
                ))
                .splineToLinearHeading(new Pose2d(-34, -12.5, Math.toRadians(0)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-57, -12.5, Math.toRadians(0)), Math.toRadians(180))
                .afterDisp(3, new ParallelAction(
                        intake.powerOn(),
                        intake.angle(3),     // Take two pixels. Remaining after: 2
                        intake.sensingOn()
                ))
                .build();
        Action TRAJ4_LeftBackboardToStack = drive.actionBuilder(leftBackboardPose)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(25, -12.5, Math.toRadians(0)), Math.toRadians(180))
                .afterDisp(3, new SequentialAction(
                        new ParallelAction(
                                outtake.latch("closed"),
                                outtake.yaw(DefVal.yaw0)
                        ),
                        new ParallelAction(
                                outtake.pivot(DefVal.pivot0),
                                outtake.roll(DefVal.roll0)
                        ),
                        outtake.runToPosition(HardwareMapping.liftHeight.GROUND)
                ))
                .splineToLinearHeading(new Pose2d(-34, -12.5, Math.toRadians(0)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-57, -12.5, Math.toRadians(0)), Math.toRadians(180))
                .afterDisp(3, new ParallelAction(
                        intake.powerOn(),
                        intake.angle(3),     // Take two pixels. Remaining after: 2
                        intake.sensingOn()
                ))
                .build();
        Action TRAJ4_RightBackboardToStack = drive.actionBuilder(rightBackboardPose)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(25, -12.5, Math.toRadians(0)), Math.toRadians(180))
                .afterDisp(3, new SequentialAction(
                        new ParallelAction(
                                outtake.latch("closed"),
                                outtake.yaw(DefVal.yaw0)
                        ),
                        new ParallelAction(
                                outtake.pivot(DefVal.pivot0),
                                outtake.roll(DefVal.roll0)
                        ),
                        outtake.runToPosition(HardwareMapping.liftHeight.GROUND)
                ))
                .splineToLinearHeading(new Pose2d(-34, -12.5, Math.toRadians(0)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-57, -12.5, Math.toRadians(0)), Math.toRadians(180))
                .afterDisp(3, new ParallelAction(
                        intake.powerOn(),
                        intake.angle(3),     // Take two pixels. Remaining after: 2
                        intake.sensingOn()
                ))
                .build();

        Action TRAJ5_StackToLeftBackboard = drive.actionBuilder(stackUpperPose)
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-34, -58.5, Math.toRadians(0)), Math.toRadians(0))
                .afterDisp(3, intake.angle(6))          // Higher intake to not get pixels
                .splineToLinearHeading(new Pose2d(25, -58.5, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(leftBackboardPose, Math.toRadians(0))
                .afterDisp(10, new ParallelAction(
                        new SequentialAction(
                                new ParallelAction(
                                        outtake.runToPosition(HardwareMapping.liftHeight.LOW),
                                        outtake.pivot(DefVal.pivot60),
                                        outtake.roll(DefVal.roll60)
                                ),
                                outtake.yaw(DefVal.yaw90),
                                outtake.latch("open")
                        ),
                        intake.sensingOff()
                ))
                .build();

        Action TRAJ6_LeftBackboardToStack = drive.actionBuilder(leftBackboardPose)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(25, -58.5, Math.toRadians(0)), Math.toRadians(180))
                .afterDisp(3, new SequentialAction(
                        outtake.yaw(DefVal.yaw0),
                        outtake.latch("closed"),
                        outtake.runToPosition(HardwareMapping.liftHeight.GROUND)
                ))
                .splineToLinearHeading(new Pose2d(-34, -58.5, Math.toRadians(0)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(stackUpperPose.position, Math.toRadians(0)), Math.toRadians(180))
                .afterDisp(3, new ParallelAction(
                        intake.powerOn(),
                        intake.angle(1),     // Take two pixels. Remaining after: 0
                        intake.sensingOn()
                ))
                .build();

        Action TRAJ7_ParkRight = drive.actionBuilder(leftBackboardPose)
                .setReversed(false)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(49, -22, Math.toRadians(0)), Math.toRadians(90))
                .afterDisp(0.1, new SequentialAction(
                        new ParallelAction(
                                outtake.yaw(0),
                                outtake.latch("closed"),
                                outtake.bottomHook("open"),
                                outtake.upperHook("open"),
                                intake.sensingOff(),
                                intake.angle(1)
                        ),
                        outtake.pivot(DefVal.pivot0),
                        outtake.roll(DefVal.roll0),
                        outtake.runToPosition(HardwareMapping.liftHeight.GROUND)
                ))
                .splineToLinearHeading(new Pose2d(59, -10, Math.toRadians(0)), Math.toRadians(0))
                .build();

        waitForStart();

        if(isStopRequested()) return;

        Pose2d nextPose;
        Action TRAJ2_LineToStack;
        switch (elementPosition) {
            case "left":
                Actions.runBlocking(auto.followTrajectoryAndStop(TRAJ1_LeftLine));
                nextPose = new Pose2d(-43, -33, Math.toRadians(110));
                TRAJ2_LineToStack = drive.actionBuilder(nextPose)
                        .setReversed(true)
                        .splineTo(new Vector2d(-52, -40), Math.toRadians(180))
                        .splineToConstantHeading(stackMiddlePose.position, Math.toRadians(90))
                        .afterDisp(1, new ParallelAction(
                                intake.powerOn(),
                                intake.angle(5),          // Take one pixel. Remaining after: 4
                                intake.sensingOn()
                        ))
                        .build();
                break;
            case "middle":
                Actions.runBlocking(auto.followTrajectoryAndStop(TRAJ1_MiddleLine));
                nextPose = new Pose2d(-32.5, -33, Math.toRadians(80));
                TRAJ2_LineToStack = drive.actionBuilder(nextPose)
                        .setReversed(true)
                        .splineTo(new Vector2d(-40, -38), Math.toRadians(180))
                        .splineToConstantHeading(stackMiddlePose.position, Math.toRadians(180))
                        .afterDisp(1, new ParallelAction(
                                intake.powerOn(),
                                intake.angle(5),          // Take one pixel. Remaining after: 4
                                intake.sensingOn()
                        ))
                        .build();
                break;
            default:
                Actions.runBlocking(auto.followTrajectoryAndStop(TRAJ1_RightLine));
                nextPose = new Pose2d(-30, -33, Math.toRadians(60));
                TRAJ2_LineToStack = drive.actionBuilder(nextPose)
                        .setReversed(true)
                        .splineTo(new Vector2d(-40, -36), Math.toRadians(180))
                        .splineToConstantHeading(stackMiddlePose.position, Math.toRadians(180))
                        .afterDisp(1, new ParallelAction(
                                intake.powerOn(),
                                intake.angle(5),          // Take one pixel. Remaining after: 4
                                intake.sensingOn()
                        ))
                        .build();
                break;
        }

        while (opModeIsActive() && !isStopRequested()){
            // Finite state
            switch (currentTraj){
                case TRAJ1_StartToLine:
                    if(!auto.isTrajGoing){
                        currentTraj = traj.TRAJ2_MiddleLineToStack;
                        Actions.runBlocking(auto.followTrajectoryAndStop(TRAJ2_LineToStack));
                    }
                    break;
                case TRAJ2_MiddleLineToStack:
                    if(!auto.isTrajGoing){
                        currentTraj=traj.TRAJ3_StackToMiddleBackboard;
                        if(elementPosition.equals("middle")){
                            Actions.runBlocking(auto.followTrajectoryAndStop(TRAJ3_StackToMiddleBackboard));
                        } else if(elementPosition.equals("left")){
                            Actions.runBlocking(auto.followTrajectoryAndStop(TRAJ3_StackToLeftBackboard));
                        } else {
                            Actions.runBlocking(auto.followTrajectoryAndStop(TRAJ3_StackToRightBackboard));
                        }
                    }
                    break;
                case TRAJ3_StackToMiddleBackboard:
                    if(!auto.isTrajGoing){
                        Actions.runBlocking(new ParallelAction(
                                outtake.bottomHook("open"),
                                outtake.upperHook("open")
                        ));
                        sleep(200);
                        //Just in case, stop the intake and start it again in the traj
                        currentTraj = traj.TRAJ4_MiddleBackboardToStack;
                        Action traj4 = TRAJ4_RightBackboardToStack;
                        if(elementPosition.equals("left")) traj4=TRAJ4_LeftBackboardToStack;
                        else if(elementPosition.equals("middle")) traj4=TRAJ4_MiddleBackboardToStack;
                        Actions.runBlocking(new ParallelAction(
                                auto.followTrajectoryAndStop(traj4),
                                intake.stop()
                        ));
                    }
                    break;
                case TRAJ4_MiddleBackboardToStack:
                case TRAJ6_LeftBackboardToStack:
                    if(!auto.isTrajGoing){
                        currentTraj = traj.TRAJ5_StackToLeftBackboard;
                        Actions.runBlocking(auto.followTrajectoryAndStop(TRAJ5_StackToLeftBackboard));
                    }
                    break;
                case TRAJ5_StackToLeftBackboard:
                    if(!auto.isTrajGoing){
                        cycleCounter++;
                        Actions.runBlocking(new ParallelAction(
                                outtake.bottomHook("open"),
                                outtake.upperHook("open")
                        ));
                        sleep(200);
                        if(cycleCounter == 1) {                        // If on the first cycle of right
                            currentTraj = traj.TRAJ6_LeftBackboardToStack;
                            Actions.runBlocking(new ParallelAction(
                                    auto.followTrajectoryAndStop(TRAJ6_LeftBackboardToStack),// Just in case, stop the intake
                                    intake.stop()                      // then start it again in the trajectory
                            ));
                        }
                        else {                                         // If on the 2nd cycle of right
                            currentTraj = traj.TRAJ7_ParkRight;
                            Actions.runBlocking(new ParallelAction(
                                    auto.followTrajectoryAndStop(TRAJ7_ParkRight),
                                    intake.stop()
                            ));
                        }
                    }
                    break;
                case TRAJ7_ParkRight:
                    if(!auto.isTrajGoing){
                        currentTraj = traj.IDLE;
                    }
                    break;
                case IDLE:
                    break;
            }

            /* Transfer last pose to TeleOP */
            Pose2d currentPose = drive.pose;
            PoseTransfer.currentPose = currentPose;
            PoseTransfer.glisieraTicks1 = robot.slideMotorRight.getCurrentPosition();
            PoseTransfer.glisieraTicks2 = robot.slideMotorLeft.getCurrentPosition();

            drive.updatePoseEstimate();

            telemetry.addData("x", currentPose.position.x);
            telemetry.addData("y", currentPose.position.y);
            telemetry.addData("heading", currentPose.heading);
            telemetry.addData("current traj: ", currentTraj.trajName);
            telemetry.addData("intakeSensingOnline: ", intake.isSensingOnline());
            telemetry.addData("isTrajGoing: ", auto.isTrajGoing);
        }
    }
}
