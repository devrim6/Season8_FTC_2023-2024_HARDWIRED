package org.firstinspires.ftc.teamcode.Autonomous.Red.PixelParking;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareMapping;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.cameraStuff.cameraHW;

@Autonomous(name = "AutoWithCamera-Red-jos", group ="Iuliu")
public class AutoRedWcamera_jos extends LinearOpMode {
    HardwareMapping robot = new HardwareMapping();
    HardwareMapping.Intake intake = robot.new Intake();
    HardwareMapping.Outtake outtake = robot.new Outtake();
    HardwareMapping.Auto auto = robot.new Auto();
    cameraHW camera = new cameraHW();
    Pose2d StartPose = new Pose2d(11, -66 , Math.toRadians(90));
    String PropZone="Middle";
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, StartPose );
        robot.init(hardwareMap);
        robot.resetEncoderAuto();
        camera.initTeamPropCamera("RED");
        PropZone=camera.isPointInsideRect();

        waitForStart();

        if(isStopRequested())
            return;
       /* switch (PropZone){
            case "Left":
                Actions.runBlocking(auto.followTrajectoryAndStop(TeamPropZoneLeft));
                break;
            case "Middle":
                Actions.runBlocking(auto.followTrajectoryAndStop(TeamPropZoneMiddle));
                break;
            case "Right":
                Actions.runBlocking(auto.followTrajectoryAndStop(TeamPropZoneRight));
                break;
        }*/
    }
}
