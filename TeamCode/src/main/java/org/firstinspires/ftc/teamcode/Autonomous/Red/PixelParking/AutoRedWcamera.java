package org.firstinspires.ftc.teamcode.Autonomous.Red.PixelParking;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.teamcode.Autonomous.ActionStorage;
import org.firstinspires.ftc.teamcode.HardwareMapping;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.cameraStuff.cameraHW;

@Autonomous(name = "AutoWithCamera-Red-sus", group = "Iuliu")
//TODO:autonomie de langa backdrop rosu
public class AutoRedWcamera extends LinearOpMode {

    HardwareMapping robot = new HardwareMapping();
    HardwareMapping.Intake intake = robot.new Intake();
    HardwareMapping.Outtake outtake = robot.new Outtake();
    HardwareMapping.Auto auto = robot.new Auto();
    cameraHW camera = new cameraHW();
    Pose2d StartPose = new Pose2d(11, -66 , Math.toRadians(90));
    String PropZone="Middle";
    ActionStorage actiuni = new ActionStorage(intake, outtake);

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, StartPose );
        robot.init(hardwareMap);
        robot.resetEncoderAuto();

        Action TeamPropZoneRight = drive.actionBuilder(StartPose)
                .splineToLinearHeading(new Pose2d(-30 + 2*24, -33, Math.toRadians(60)), Math.toRadians(60))
                .setReversed(true)
                .afterDisp(2, intake.angle(6))//sau 1
                .splineToLinearHeading(new Pose2d(49, -30, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(49, -42, Math.toRadians(0)), Math.toRadians(0))
                .afterDisp(4, actiuni.pixelToMiddle)
                .build();
        
        Action TeamPropZoneMiddle = drive.actionBuilder(StartPose)
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(10, -30, Math.toRadians(110)), Math.toRadians(110))
                .setTangent(Math.toRadians(0))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(21, -43, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(48, -30, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(0.7) //pune pixelul
                .splineToLinearHeading(new Pose2d(2,-59,Math.toRadians(0)),Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-54,-59,Math.toRadians(0)),Math.toRadians(180))
                .build();

        Action TeamPropZoneLeft = drive.actionBuilder(StartPose)
                .setReversed(false)
                .splineToSplineHeading(new Pose2d(23.00, -36.00, Math.toRadians(80.00)),Math.toRadians(80))
                .setTangent(Math.toRadians(0))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(48, -30, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(0.7) //pune pixelul
                .splineToLinearHeading(new Pose2d(2,-59,Math.toRadians(0)),Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-54,-59,Math.toRadians(0)),Math.toRadians(180))
                .build();


        camera.initTeamPropCamera("RED");
        PropZone=camera.isPointInsideRect();

                waitForStart();

                if(isStopRequested())
                    return;
                switch (PropZone){
                    case "Left":
                        Actions.runBlocking(auto.followTrajectoryAndStop(TeamPropZoneLeft));
                        break;
                    case "Middle":
                        Actions.runBlocking(auto.followTrajectoryAndStop(TeamPropZoneMiddle));
                        break;
                    case "Right":
                        Actions.runBlocking(auto.followTrajectoryAndStop(TeamPropZoneRight));
                        break;
                }

    }
}
