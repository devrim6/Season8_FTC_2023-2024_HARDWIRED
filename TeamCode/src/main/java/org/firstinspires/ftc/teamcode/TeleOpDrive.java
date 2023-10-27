package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp (name = "TeleOpDrive")
public class TeleOpDrive extends LinearOpMode {
    /* Init whatever you need */
    HardwareMapping robot = new HardwareMapping();
    HardwareMapping.Intake intake = new HardwareMapping.Intake();
    HardwareMapping.Outtake outtake = new HardwareMapping.Outtake();
    enum mode {
        TELEOP,
        HEADING_LOCK
    }
    mode currentMode = mode.TELEOP;
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));

        // Variables
        double triggerSlowdown = gamepad2.right_trigger;

        //Funky time
        waitForStart();
        while(opModeIsActive() && !isStopRequested()){
            switch(currentMode){
                case TELEOP:

                    drive.setDrivePowers(new PoseVelocity2d( // Slowdown by pressing right trigger, is gradual
                            new Vector2d(
                                    -gamepad1.left_stick_y/(1+triggerSlowdown),
                                    -gamepad1.left_stick_x/(1+triggerSlowdown)
                            ),
                            -gamepad1.right_stick_x/(1+triggerSlowdown*3)
                    ));

                    drive.updatePoseEstimate();

                    telemetry.addData("x", drive.pose.position.x);
                    telemetry.addData("y", drive.pose.position.y);
                    telemetry.addData("heading", drive.pose.heading);
                    telemetry.update();



                case HEADING_LOCK:
            }
        }
    }
}
