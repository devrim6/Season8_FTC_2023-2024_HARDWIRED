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
    HardwareMapping.Intake intake = robot.new Intake();
    HardwareMapping.Outtake outtake = robot.new Outtake();
    enum mode {
        TELEOP,
        HEADING_LOCK
    }
    mode currentMode = mode.TELEOP;
    @Override
    public void runOpMode() throws InterruptedException {
        //TODO: the to do list
        //masurare amperaj motoare glisiera pt determinare gear ratio optim la viteza, afaik max 4a, uitate pe spec sheet la rev
        //recalibrare pozitie cu camera april tags la backboard, triunghiulare maybe? look into it, experimenteaza cate tag-uri se vad intrun frame
        //daca localizarea ii accurate la sfarsit (cat de cat) incearca sa faci o traiectorie pt locul de lansat avion daca le trebuie la driveri, si pt hanging daca nu
        //look into angular velocity tuning pt rotiri mai rapide si consistente
        //dupa ce ii gata o autonomie, incearca sa bagi actiunile intrun alt fisier pt easier use
        //led-uri pe cuva/text pe consola sa zica ce tip de pixel ii in care parte a robotului, gen culoare sau daca exista
        //implementare senzori pt pixeli in cuva, daca sunt 2 in cuva driver 1/2 numai reverse poate da la intake motors/roller
        //centrare pe april tag la backboard in teleop, depinde daca ne ajuta sau nu
        robot.init(hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap, PoseTransfer.currentPose);
        // Init motors/servos/etc


        // Variables
        double triggerSlowdown = gamepad2.right_trigger;

        //Funky time
        waitForStart();
        while(opModeIsActive() && !isStopRequested()){
            switch(currentMode){
                case TELEOP:

                    drive.setDrivePowers(new PoseVelocity2d( // Slowdown by pressing right trigger, is gradual
                            new Vector2d(
                                    -gamepad2.left_stick_y/(1+triggerSlowdown),
                                    -gamepad2.left_stick_x/(1+triggerSlowdown)),
                            -gamepad2.right_stick_x/(1+triggerSlowdown*3)
                    ));
                    break;
                case HEADING_LOCK:

                    drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(-gamepad2.left_stick_y/(1+triggerSlowdown),
                                    -gamepad2.left_stick_x/(1+triggerSlowdown)),
                            Math.toRadians(90)
                    ));
                    break;
            }

            drive.updatePoseEstimate();

            if(gamepad2.dpad_left) Actions.runBlocking(outtake.runToPosition("ground"));
            if(gamepad2.dpad_up) Actions.runBlocking(outtake.runToPosition("third"), outtake.pivot(0.4, -0.4));
            if(gamepad2.dpad_down) Actions.runBlocking(outtake.runToPosition("first"), outtake.pivot(0.4, -0.4));
            if(gamepad2.dpad_right) Actions.runBlocking(outtake.runToPosition("second"), outtake.pivot(0.4, -0.4));

            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading", drive.pose.heading);
            telemetry.addLine("---DEBUG---");
            telemetry.update();
        }
    }
}
