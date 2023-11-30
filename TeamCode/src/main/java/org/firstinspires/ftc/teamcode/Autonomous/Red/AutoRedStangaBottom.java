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
import org.firstinspires.ftc.teamcode.Variables.DefVal;

@Autonomous(name = "AutoRedStangaBottom")
public class AutoRedStangaBottom extends LinearOpMode {
    /* Init whatever you need */
    HardwareMapping robot = new HardwareMapping();
    HardwareMapping.Intake intake = robot.new Intake();
    HardwareMapping.Outtake outtake = robot.new Outtake();
    enum traj {
        TRAJ1_StartToLine("TRAJ1_StartToLine"),
        TRAJ2_MiddleLineToStack("TRAJ2_MiddleLineToStack"),

        TRAJ3_StackToMiddleBackboard("TRAJ3_StackToMiddleBackboard"),
        TRAJ4_MiddleBackboardToStack("TRAJ4_MiddleBackboardToStack"),

        TRAJ5_StackToRightBackboard("TRAJ5_StackToRightBackboard"),
        TRAJ6_RightBackboardToStack("TRAJ6_RightBackboardToStack"),

        TRAJ7_ParkRight("TRAJ7_ParkRight"),
        IDLE("IDLE");

        public final String trajName;

        traj(String trajName){
            this.trajName = trajName;
        }
    }

    Pose2d stackPose = new Pose2d(-57, -36, Math.toRadians(180)),
           middleBackboardPose = new Pose2d(49, -36, Math.toRadians(0)),
           rightBackboardPose = new Pose2d(49, -42, Math.toRadians(0));

    double cycleCounter = 0;

    String elementPosition = "middle";
    traj currentTraj = traj.TRAJ1_StartToLine;
    public void runOpMode() throws InterruptedException{
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(58, -34.5, Math.toRadians(270)));
        robot.init(hardwareMap);
        robot.resetEncoder();

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

        Action TRAJ3_StackToMiddleBackboard = drive.actionBuilder(stackPose)
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-34, -58.5, Math.toRadians(0)), Math.toRadians(0))
                .afterDisp(3, intake.angle(6))              // Higher intake to not get pixels
                .splineToSplineHeading(new Pose2d(-10, -58.5, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(25, -58.5, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(middleBackboardPose, Math.toRadians(0))
                .afterDisp(10, new ParallelAction(
                        new SequentialAction(
                                new ParallelAction(
                                        outtake.runToPosition(HardwareMapping.liftHeight.LOW),
                                        outtake.pivot(DefVal.pivot60_1, DefVal.pivot60_2),
                                        outtake.roll(DefVal.roll60_1, DefVal.roll60_2)
                                ),
                                outtake.yaw(DefVal.yaw90),
                                outtake.latch("open")
                        ),
                        intake.sensingOff()
                ))
                .build();

        Action TRAJ4_MiddleBackboardToStack = drive.actionBuilder(middleBackboardPose)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(25, -58.5, Math.toRadians(0)), Math.toRadians(180))
                .afterDisp(3, new SequentialAction(
                        new ParallelAction(
                                outtake.latch("closed"),
                                outtake.yaw(DefVal.yaw0)
                        ),
                        new ParallelAction(
                                outtake.pivot(DefVal.pivot0_1, DefVal.pivot0_2),
                                outtake.roll(DefVal.roll0_1, DefVal.roll0_2)
                        ),
                        outtake.runToPosition(HardwareMapping.liftHeight.GROUND)
                ))
                .splineToLinearHeading(new Pose2d(-10, -58.5, Math.toRadians(0)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-34, -58.5, Math.toRadians(0)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(stackPose.position, Math.toRadians(0)), Math.toRadians(180))
                .afterDisp(3, new ParallelAction(
                        intake.powerOn(),
                        intake.angle(3),     // Take two pixels. Remaining after: 2
                        intake.sensingOn()
                ))
                .build();

        Action TRAJ5_StackToRightBackboard = drive.actionBuilder(stackPose)
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-34, -58.5, Math.toRadians(0)), Math.toRadians(0))
                .afterDisp(3, intake.angle(6))          // Higher intake to not get pixels
                .splineToSplineHeading(new Pose2d(-10, -58.5, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(25, -58.5, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(rightBackboardPose, Math.toRadians(0))
                .afterDisp(10, new ParallelAction(
                        new SequentialAction(
                                new ParallelAction(
                                        outtake.runToPosition(HardwareMapping.liftHeight.LOW),
                                        outtake.pivot(DefVal.pivot60_1, DefVal.pivot60_2),
                                        outtake.roll(DefVal.roll60_1, DefVal.roll60_2)
                                ),
                                outtake.yaw(DefVal.yaw90),
                                outtake.latch("open")
                        ),
                        intake.sensingOff()
                ))
                .build();

        Action TRAJ6_RightBackboardToStack = drive.actionBuilder(rightBackboardPose)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(25, -58.5, Math.toRadians(0)), Math.toRadians(180))
                .afterDisp(3, new SequentialAction(
                        outtake.yaw(DefVal.yaw0),
                        outtake.latch("closed"),
                        outtake.runToPosition(HardwareMapping.liftHeight.GROUND)
                ))
                .splineToLinearHeading(new Pose2d(-10, -58.5, Math.toRadians(0)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-34, -58.5, Math.toRadians(0)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(stackPose.position, Math.toRadians(0)), Math.toRadians(180))
                .afterDisp(3, new ParallelAction(
                        intake.powerOn(),
                        intake.angle(1),     // Take two pixels. Remaining after: 0
                        intake.sensingOn()
                ))
                .build();

        Action TRAJ7_ParkRight = drive.actionBuilder(rightBackboardPose)
                .setReversed(false)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(49, -55, Math.toRadians(0)), Math.toRadians(-90))
                .afterDisp(0.1, new SequentialAction(
                        new ParallelAction(
                                outtake.yaw(0),
                                outtake.latch("closed"),
                                outtake.bottomHook("open"),
                                outtake.upperHook("open"),
                                intake.sensingOff(),
                                intake.angle(1)
                        ),
                        outtake.pivot(DefVal.pivot0_1, DefVal.pivot0_2),
                        outtake.roll(DefVal.roll0_1, DefVal.roll0_2),
                        outtake.runToPosition(HardwareMapping.liftHeight.GROUND)
                ))
                .splineToLinearHeading(new Pose2d(57.5, -60, Math.toRadians(0)), Math.toRadians(0))
                .build();

        waitForStart();

        if(isStopRequested()) return;

        Pose2d nextPose;
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
                        intake.angle(5),          // Take one pixel. Remaining after: 4
                        intake.sensingOn()
                ))
                .build();

        while (opModeIsActive() && !isStopRequested()){
            // Finite state
            switch (currentTraj){
                case TRAJ1_StartToLine:
                    if(!drive.isBusy()){
                        currentTraj = traj.TRAJ2_MiddleLineToStack;
                        Actions.runBlocking(TRAJ2_MiddleLineToStack);
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
                        Actions.runBlocking(new ParallelAction(
                                outtake.bottomHook("open"),
                                outtake.upperHook("open")
                        ));
                        sleep(200);
                        Actions.runBlocking(new ParallelAction(
                                TRAJ4_MiddleBackboardToStack,      // Just in case, stop the intake
                                intake.stop()                      // then start it again in the trajectory
                        ));
                    }
                    break;
                case TRAJ4_MiddleBackboardToStack:
                case TRAJ6_RightBackboardToStack:
                    if(!drive.isBusy()){
                        currentTraj = traj.TRAJ5_StackToRightBackboard;
                        Actions.runBlocking(TRAJ5_StackToRightBackboard);
                    }
                    break;
                case TRAJ5_StackToRightBackboard:
                    if(!drive.isBusy()){
                        cycleCounter++;
                        Actions.runBlocking(new ParallelAction(
                                outtake.bottomHook("open"),
                                outtake.upperHook("open")
                        ));
                        sleep(200);
                        if(cycleCounter == 1) {                        // If on the first cycle of right
                            currentTraj = traj.TRAJ6_RightBackboardToStack;
                            Actions.runBlocking(new ParallelAction(
                                    TRAJ6_RightBackboardToStack,       // Just in case, stop the intake
                                    intake.stop()                      // then start it again in the trajectory
                            ));
                        }
                        else {                                         // If on the 2nd cycle of right
                            currentTraj = traj.TRAJ7_ParkRight;
                            Actions.runBlocking(new ParallelAction(
                                    TRAJ7_ParkRight,
                                    intake.stop()
                            ));
                        }
                    }
                    break;
                case TRAJ7_ParkRight:
                    if(!drive.isBusy()){
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
            telemetry.addData("intakeSensingOnline: ", intake.isSensingOnline());
        }
    }
}
