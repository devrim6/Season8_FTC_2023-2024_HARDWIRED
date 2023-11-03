package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;

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

    /**
     *  DRIVER 1
     *   X         - Power on/off INTAKE
     *   Y         - Engage hooks on/off
     *   B         -
     *   A         -
     *   Left/Right stick - Base controls
     *   DPAD left     - Lift ground
     *   DPAD down     - Lift 1st level
     *   DPAD right    - Lift 2nd level
     *   DPAD up       - Lift 3rd level
     *
     *   DRIVER 2
     *   X         - Power on/off INTAKE
     *   Y         - Engage hooks on/off
     *   B         -
     *   A         -
     *   Left stick Y - manual slide control
     *   Left stick X - manual outtake pitch (keep 60 degree angle)
     *   DPAD left     - Lift ground
     *   DPAD down     - Lift 1st level
     *   DPAD right    - Lift 2nd level
     *   DPAD up       - Lift 3rd level
     *
     *       _=====_                               _=====_
     *      / _____ \                             / _____ \
     *    +.-'_____'-.---------------------------.-'_____'-.+
     *   /   |     |  '.                       .'  |  _  |   \
     *  / ___| /|\ |___ \    BACK     START   / ___| (Y) |___ \
     * / |      |      | ;                   ; |              | ;
     * | | <---   ---> | |                   | |(X)       (B) | |
     * | |___   |   ___| ;  MODE             ; |___        ___| ;
     * |\    | \|/ |    /  _              _   \    | (A) |    /|
     * | \   |_____|  .','" "',        ,'" "', '.  |_____|  .' |
     * |  '-.______.-' /       \      /       \  '-._____.-'   |
     * |               |       |------|       |                |
     * |              /\       /      \       /\               |
     * |             /  '.___.'        '.___.'  \              |
     * |            /                            \             |
     *  \          /                              \           /
     *   \________/                                \_________/
     */
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
        robot.gamepadInit(gamepad1, gamepad2);
        MecanumDrive drive = new MecanumDrive(hardwareMap, PoseTransfer.currentPose);
        // Init motors/servos/etc


        // Variables
        double triggerSlowdown = gamepad2.right_trigger;
        //TODO: transfer hook state between auto in case auto fails
        boolean isIntakePowered = false, intakeManualControl = false, areHooksEngaged=false;

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


            // Gamepad controls

            //Slide controls
            //Driver 1 and 2
            if(gamepad2.dpad_left || gamepad1.dpad_left) Actions.runBlocking(outtake.runToPosition("ground"));
            if(gamepad2.dpad_up || gamepad1.dpad_up) Actions.runBlocking(outtake.runToPosition("third"), outtake.pivot(0.4, -0.4));
            if(gamepad2.dpad_down || gamepad1.dpad_down) Actions.runBlocking(outtake.runToPosition("first"), outtake.pivot(0.4, -0.4));
            if(gamepad2.dpad_right || gamepad1.dpad_right) Actions.runBlocking(outtake.runToPosition("second"), outtake.pivot(0.4, -0.4));

            //Hook engage control
            if(gamepad2.y || gamepad1.y){ // Double tap prevention, maybe works?
                if(robot.gamepad1Ex.stateJustChanged(GamepadKeys.Button.Y) || robot.gamepad2Ex.stateJustChanged(GamepadKeys.Button.Y)) {
                    areHooksEngaged=!areHooksEngaged;
                    if(areHooksEngaged) Actions.runBlocking(outtake.bottomHook("closed"), outtake.upperHook("closed"));
                    else Actions.runBlocking(outtake.bottomHook("open"), outtake.upperHook("open"));
                }
            }

            //Intake controls
            if(gamepad2.x || gamepad1.x){ // Double tap prevention, maybe works?
                if(robot.gamepad1Ex.stateJustChanged(GamepadKeys.Button.X) || robot.gamepad2Ex.stateJustChanged(GamepadKeys.Button.X)){
                    isIntakePowered=!isIntakePowered;
                    if(isIntakePowered){
                        Actions.runBlocking(intake.powerOn());
                        intakeManualControl = true;
                    } else {
                        Actions.runBlocking(intake.stop());
                    }
                }
            }


            //If intake is running and two pixels are already in, stop the intake
            //TODO: implement check for manual reverse in case 3rd pixel comes in (kinda did with intakemanualcontrol? needs testing)
            if(isIntakePowered && intakeManualControl){
                if(!Objects.equals(robot.checkColorRange("upper"), "none")&& !Objects.equals(robot.checkColorRange("bottom"), "none")) {
                    Actions.runBlocking(intake.stop());
                    Actions.runBlocking(outtake.bottomHook("closed"), outtake.upperHook("closed"));
                    areHooksEngaged=true;
                    isIntakePowered = false;
                    intakeManualControl = false;
                }
            }

            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading", drive.pose.heading);
            telemetry.addLine("---DEBUG---");
            telemetry.addData("intakeManualControl: ", intakeManualControl);
            telemetry.addData("isIntakePowered: ", isIntakePowered);
            telemetry.addData("areHooksEngaged: ", areHooksEngaged);
            telemetry.update();
        }
    }
}
