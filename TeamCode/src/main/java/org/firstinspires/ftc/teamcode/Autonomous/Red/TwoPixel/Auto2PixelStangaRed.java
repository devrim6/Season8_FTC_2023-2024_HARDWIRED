package org.firstinspires.ftc.teamcode.Autonomous.Red.TwoPixel;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.ActionStorage;
import org.firstinspires.ftc.teamcode.HardwareMapping;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PoseTransfer;
import org.firstinspires.ftc.teamcode.Variables.DefVal;

@Autonomous(group = "Auto Red", name = "Auto2PixelStangaRed")
public class Auto2PixelStangaRed extends LinearOpMode {
    /* Init whatever you need */
    HardwareMapping robot = new HardwareMapping();
    HardwareMapping.Intake intake = robot.new Intake();
    HardwareMapping.Outtake outtake = robot.new Outtake();
    HardwareMapping.Auto auto = robot.new Auto();
    enum traj {
        TRAJ1_StartToLine("TRAJ1_StartToLine"),
        TRAJ2_LineToBackboard("TRAJ2_LineToBackboard"),
        TRAJ3_Park("TRAJ3_Park"),
        IDLE("IDLE");

        public final String trajName;

        traj(String trajName){
            this.trajName = trajName;
        }
    }

    Pose2d stackMiddlePose = new Pose2d(-57, -25, Math.toRadians(180)),
            stackUpperPose = new Pose2d(-57, -12.5, Math.toRadians(0)),
            stackBottomPose = new Pose2d(-57, -36, Math.toRadians(0)),
            middleBackboardPose = new Pose2d(49, -36, Math.toRadians(0)),
            rightBackboardPose = new Pose2d(49, -42, Math.toRadians(0)),
            leftBackboardPose = new Pose2d(49, -30, Math.toRadians(0));

    double cycleCounter = 0;

    String elementPosition = "middle";
    traj currentTraj = traj.TRAJ1_StartToLine;

    ActionStorage actionStorage = new ActionStorage(intake, outtake);

    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-34.5 + 2 * 24, -58, Math.toRadians(90)));
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

        Action TRAJ3_Park = drive.actionBuilder(leftBackboardPose)
                .setReversed(false)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(49, -18, Math.toRadians(0)), Math.toRadians(90))
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
                        outtake.runToPosition("ground")
                ))
                .build();;

        waitForStart();

        if(isStopRequested()) return;

        Pose2d nextPose;
        Action TRAJ2_LineToBackboard;
        switch (elementPosition) {
            case "left":
                Actions.runBlocking(auto.followTrajectoryAndStop(TRAJ1_LeftLine));
                nextPose = new Pose2d(-43, -33, Math.toRadians(110));
                TRAJ2_LineToBackboard = drive.actionBuilder(nextPose)
                        .setReversed(false)
                        .splineToLinearHeading(new Pose2d(-34, -12.5, Math.toRadians(0)), Math.toRadians(0))
                        .afterDisp(3, intake.angle(6))              // Higher intake to not get pixels
                        .splineToLinearHeading(new Pose2d(25, -12.5, Math.toRadians(0)), Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(49, -30, Math.toRadians(0)), Math.toRadians(0))
                        .afterDisp(5, actionStorage.pixelToLow)
                        .build();
                break;
            case "middle":
                Actions.runBlocking(auto.followTrajectoryAndStop(TRAJ1_MiddleLine));
                nextPose = new Pose2d(-32.5, -33, Math.toRadians(80));
                TRAJ2_LineToBackboard = drive.actionBuilder(nextPose)
                        .setReversed(false)
                        .splineToLinearHeading(new Pose2d(-34, -12.5, Math.toRadians(0)), Math.toRadians(0))
                        .afterDisp(3, intake.angle(6))              // Higher intake to not get pixels
                        .splineToLinearHeading(new Pose2d(25, -12.5, Math.toRadians(0)), Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(49, -36, Math.toRadians(0)), Math.toRadians(0))
                        .afterDisp(10, actionStorage.pixelToLow)
                        .build();
                break;
            default:
                Actions.runBlocking(auto.followTrajectoryAndStop(TRAJ1_RightLine));
                nextPose = new Pose2d(-30, -33, Math.toRadians(60));
                TRAJ2_LineToBackboard = drive.actionBuilder(nextPose)
                        .setReversed(false)
                        .splineToLinearHeading(new Pose2d(-34, -12.5, Math.toRadians(0)), Math.toRadians(0))
                        .afterDisp(3, intake.angle(6))              // Higher intake to not get pixels
                        .splineToLinearHeading(new Pose2d(25, -12.5, Math.toRadians(0)), Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(49, -42, Math.toRadians(0)), Math.toRadians(0))
                        .afterDisp(5, actionStorage.pixelToLow)
                        .build();
                break;
        }

        while (opModeIsActive() && !isStopRequested()){

            // Finite state
            switch (currentTraj){
                case TRAJ1_StartToLine:
                    if(!auto.isTrajGoing){
                        currentTraj = traj.TRAJ2_LineToBackboard;
                        Actions.runBlocking(auto.followTrajectoryAndStop(TRAJ2_LineToBackboard));
                    }
                    break;
                case TRAJ2_LineToBackboard:
                    if(!auto.isTrajGoing) {
                        Actions.runBlocking(new ParallelAction(
                                outtake.bottomHook("open"),
                                outtake.upperHook("open")
                        ));
                        sleep(200);
                        currentTraj = traj.TRAJ3_Park;
                        Actions.runBlocking(auto.followTrajectoryAndStop(TRAJ3_Park));
                    }
                    break;
                case TRAJ3_Park:
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
            telemetry.addData("cycleCounter: ", cycleCounter);
            telemetry.update();
        }
    }
}
