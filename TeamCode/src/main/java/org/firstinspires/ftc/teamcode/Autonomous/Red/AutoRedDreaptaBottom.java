//package org.firstinspires.ftc.teamcode.Autonomous.Red;
//
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.ParallelAction;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.SleepAction;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.acmerobotics.roadrunner.ftc.Actions;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.Autonomous.ActionStorage;
//import org.firstinspires.ftc.teamcode.HardwareMapping;
//import org.firstinspires.ftc.teamcode.MecanumDrive;
//import org.firstinspires.ftc.teamcode.PoseTransfer;
//import org.firstinspires.ftc.teamcode.Variables.DefVal;
//
//@Autonomous(group = "Auto Red", name = "AutoDreaptaRedBottom")
//public class AutoRedDreaptaBottom extends LinearOpMode {
//    /* Init whatever you need */
//    HardwareMapping robot = new HardwareMapping();
//    HardwareMapping.Intake intake = robot.new Intake();
//    HardwareMapping.Outtake outtake = robot.new Outtake();
//    HardwareMapping.Auto auto = robot.new Auto();
//    enum traj {
//        TRAJ1_StartToLine("TRAJ1_StartToLine"),
//        TRAJ2_LineToBackboard("TRAJ2_LineToBackboard"),
//
//        TRAJ3_BackboardToStack("TRAJ3_StackToMiddleBackboard"),
//
//        TRAJ4_StackToRightBackboard("TRAJ4_MiddleBackboardToStack"),
//
//        TRAJ5_RightBackboardToStack("TRAJ5_StackToLeftBackboard"),
//
//        TRAJ6_ParkRight("TRAJ7_ParkRight"),
//        IDLE("IDLE");
//
//        public final String trajName;
//
//        traj(String trajName){
//            this.trajName = trajName;
//        }
//    }
//
//    Pose2d  stackMiddlePose = new Pose2d(-57, -25, Math.toRadians(180)),
//            stackUpperPose = new Pose2d(-57, -12.5, Math.toRadians(0)),
//            stackBottomPose = new Pose2d(-57, -36, Math.toRadians(0)),
//            middleBackboardPose = new Pose2d(49, -36, Math.toRadians(0)),
//            rightBackboardPose = new Pose2d(49, -42, Math.toRadians(0)),
//            leftBackboardPose = new Pose2d(49, -30, Math.toRadians(0));
//
//    double cycleCounter = 0;
//
//    String elementPosition = "middle";
//    traj currentTraj = traj.TRAJ1_StartToLine;
//
//    ActionStorage actionStorage = new ActionStorage(intake, outtake);
//
//    public void runOpMode() throws InterruptedException{
//        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-34.5 + 2*24 , -58 , Math.toRadians(90)));
//        robot.init(hardwareMap);
//        robot.resetEncoderAuto();
//
//        Action TRAJ1_MiddleLine = drive.actionBuilder(drive.pose)
//                .splineToLinearHeading(new Pose2d(15.5, -33, Math.toRadians(80)), Math.toRadians(80))
//                .build();
//        Action TRAJ1_LeftLine = drive.actionBuilder(drive.pose)
//                .splineTo(new Vector2d(13.5, -53), Math.toRadians(90))
//                .splineTo(new Vector2d(4.5, -33), Math.toRadians(120))
//                .build();
//        Action TRAJ1_RightLine = drive.actionBuilder(drive.pose)
//                .splineToLinearHeading(new Pose2d(18, -33, Math.toRadians(60)), Math.toRadians(60))
//                .build();
//
//        Action TRAJ4_StackToRightBackboard = drive.actionBuilder(stackBottomPose)
//                .setReversed(false)
//                .setTangent(Math.toRadians(-70))
//                .splineToLinearHeading(new Pose2d(-34, -58.5, Math.toRadians(0)), Math.toRadians(0))
//                .afterDisp(3, intake.angle(6))              // Higher intake to not get pixels
//                .splineToSplineHeading(new Pose2d(-10, -58.5, Math.toRadians(0)), Math.toRadians(0))
//                .splineToLinearHeading(new Pose2d(25, -58.5, Math.toRadians(0)), Math.toRadians(0))
//                .afterDisp(0.1, intake.reverse())
//                .splineToLinearHeading(rightBackboardPose, Math.toRadians(0))
//                .afterDisp(5, actionStorage.pixelToMiddle)
//                .build();
//
//        Action TRAJ5_RightBackboardToStack = drive.actionBuilder(rightBackboardPose)
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(25, -58.5, Math.toRadians(0)), Math.toRadians(180))
//                .afterDisp(2, actionStorage.pixelToGround)
//                .splineToLinearHeading(new Pose2d(-10, -58.5, Math.toRadians(0)), Math.toRadians(180))
//                .splineToLinearHeading(new Pose2d(-34, -58.5, Math.toRadians(0)), Math.toRadians(180))
//                .splineToLinearHeading(new Pose2d(-57, -36, Math.toRadians(0)), Math.toRadians(110))
//                .afterDisp(3, new ParallelAction(
//                        intake.powerOn(),
//                        // Set intake level in the finite state switch
//                        intake.sensingOn()
//                ))
//                .build();
//
//        Action TRAJ6_ParkRight = drive.actionBuilder(rightBackboardPose)
//                .setReversed(false)
//                .setTangent(Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(49, -18, Math.toRadians(0)), Math.toRadians(90))
//                .afterDisp(0.1, new SequentialAction(
//                        new ParallelAction(
//                                outtake.yaw(0),
//                                outtake.latch("closed"),
//                                outtake.bottomHook("open"),
//                                outtake.upperHook("open"),
//                                intake.sensingOff(),
//                                intake.angle(1)
//                        ),
//                        outtake.pivot(DefVal.pivot0),
//                        outtake.roll(DefVal.roll0),
//                        new SleepAction(1),
//                        outtake.runToPosition("ground")
//                ))
//                .build();
//
//        waitForStart();
//
//        if(isStopRequested()) return;
//
//        Pose2d nextPose, nextNextPose=new Pose2d(0,0,0);
//        Action TRAJ2_LineToStack;
//        switch (elementPosition) {
//            case "right":
//                Actions.runBlocking(auto.followTrajectoryAndStop(TRAJ1_RightLine));
//                nextPose = new Pose2d(18, -33, Math.toRadians(60));
//                TRAJ2_LineToStack = drive.actionBuilder(nextPose)
//                        .setReversed(true)
//                        .splineToLinearHeading(rightBackboardPose, Math.toRadians(0))
//                        .afterDisp(4, actionStorage.pixelToLow)
//                        .build();
//                nextNextPose=rightBackboardPose;
//                break;
//            case "middle":
//                Actions.runBlocking(auto.followTrajectoryAndStop(TRAJ1_MiddleLine));
//                nextPose = new Pose2d(15.5, -33, Math.toRadians(80));
//                TRAJ2_LineToStack = drive.actionBuilder(nextPose)
//                        .setReversed(true)
//                        .splineToLinearHeading(middleBackboardPose, Math.toRadians(0))
//                        .afterDisp(4, actionStorage.pixelToLow)
//                        .build();
//                nextNextPose=middleBackboardPose;
//                break;
//            default:
//                Actions.runBlocking(auto.followTrajectoryAndStop(TRAJ1_LeftLine));
//                nextPose = new Pose2d(4.5, -33, Math.toRadians(120));
//                TRAJ2_LineToStack = drive.actionBuilder(nextPose)
//                        .setReversed(true)
//                        .splineToLinearHeading(leftBackboardPose, Math.toRadians(0))
//                        .afterDisp(4, actionStorage.pixelToLow)
//                        .build();
//                nextNextPose=leftBackboardPose;
//                break;
//        }
//        Action TRAJ3_BackboardToStack = drive.actionBuilder(nextNextPose)
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(25, -58.5, Math.toRadians(0)), Math.toRadians(180))
//                .afterDisp(2, actionStorage.pixelToGround)
//                .splineToLinearHeading(new Pose2d(-10, -58.5, Math.toRadians(0)), Math.toRadians(180))
//                .splineToLinearHeading(new Pose2d(-34, -58.5, Math.toRadians(0)), Math.toRadians(180))
//                .splineToLinearHeading(new Pose2d(-57, -36, Math.toRadians(0)), Math.toRadians(110))
//                .afterDisp(3, new ParallelAction(
//                        intake.powerOn(),
//                        intake.angle(5),     // Take two pixels. Remaining after: 3
//                        intake.sensingOn()
//                ))
//                .build();
//
//        while (opModeIsActive() && !isStopRequested()){
//            // Finite state
//            switch (currentTraj){
//                case TRAJ1_StartToLine:
//                    if(!auto.isTrajGoing){
//                        currentTraj = traj.TRAJ2_LineToBackboard;
//                        Actions.runBlocking(auto.followTrajectoryAndStop(TRAJ2_LineToStack));
//                    }
//                    break;
//                case TRAJ2_LineToBackboard:
//                    if(!auto.isTrajGoing){
//                        Actions.runBlocking(new ParallelAction(
//                                outtake.bottomHook("open"),
//                                outtake.upperHook("open")
//                        ));
//                        sleep(200);
//                        currentTraj=traj.TRAJ3_BackboardToStack;
//                        Actions.runBlocking(auto.followTrajectoryAndStop(TRAJ3_BackboardToStack));
//                    }
//                    break;
//                case TRAJ3_BackboardToStack:
//                case TRAJ5_RightBackboardToStack:
//                    if(!auto.isTrajGoing){
//                        currentTraj = traj.TRAJ4_StackToRightBackboard;
//                        Actions.runBlocking(auto.followTrajectoryAndStop(TRAJ4_StackToRightBackboard));
//                    }
//                    break;
//                case TRAJ4_StackToRightBackboard:
//                    if(!auto.isTrajGoing){
//                        cycleCounter++;
//                        Actions.runBlocking(new ParallelAction(
//                                outtake.bottomHook("open"),
//                                outtake.upperHook("open")
//                        ));
//                        sleep(200);
//                        if(cycleCounter==1){
//                            currentTraj = traj.TRAJ5_RightBackboardToStack;
//                            Actions.runBlocking(auto.followTrajectoryAndStop(TRAJ5_RightBackboardToStack));
//                        }
//                        else if(cycleCounter==2){
//                            currentTraj = traj.TRAJ6_ParkRight;
//                            Actions.runBlocking(auto.followTrajectoryAndStop(TRAJ6_ParkRight));
//                        }
//                    }
//                    break;
//                case TRAJ6_ParkRight:
//                    if(!auto.isTrajGoing){
//                        currentTraj = traj.IDLE;
//                    }
//                    break;
//                case IDLE:
//                    break;
//            }
//
//            /* Transfer last pose to TeleOP */
//            Pose2d currentPose = drive.pose;
//            PoseTransfer.currentPose = currentPose;
//            PoseTransfer.glisieraTicks1 = robot.slideMotorRight.getCurrentPosition();
//            PoseTransfer.glisieraTicks2 = robot.slideMotorLeft.getCurrentPosition();
//
//            drive.updatePoseEstimate();
//
//            telemetry.addData("x", currentPose.position.x);
//            telemetry.addData("y", currentPose.position.y);
//            telemetry.addData("heading", currentPose.heading);
//            telemetry.addData("current traj: ", currentTraj.trajName);
//            telemetry.addData("intakeSensingOnline: ", intake.isSensingOnline());
//            telemetry.addData("isTrajGoing: ", auto.isTrajGoing);
//            telemetry.addData("cycleCounter: ", cycleCounter);
//            telemetry.update();
//        }
//    }
//}
