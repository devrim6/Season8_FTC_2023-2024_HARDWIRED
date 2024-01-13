package org.firstinspires.ftc.teamcode.Autonomous.Red.PixelParking;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareMapping;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="AutoRedLeftMiddle")
public class AutoRedLeftMiddle extends LinearOpMode {
    HardwareMapping robot = new HardwareMapping();
    private Pose2d cPose;
    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap,new Pose2d(-34.5, -58 , Math.toRadians(90)));
        robot.init(hardwareMap);
        robot.resetEncoderAuto();
        Pose2d leftLine=new Pose2d(-39,-36,Math.toRadians(120));


        Action TrajToParking=drive.actionBuilder(cPose)
                .setReversed(false)
                //.splineToLinearHeading(new Vector2d(60,-60))
                .build();

        waitForStart();

        Actions.runBlocking(new SequentialAction(
                TrajToParking
        ));
    }
}
