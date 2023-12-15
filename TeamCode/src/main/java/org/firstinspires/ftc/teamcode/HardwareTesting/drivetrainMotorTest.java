package org.firstinspires.ftc.teamcode.HardwareTesting;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class drivetrainMotorTest extends LinearOpMode {

    private DcMotorEx leftFront, leftBack, rightFront, rightBack;
    private GamepadEx gamepadEx = new GamepadEx(gamepad2);
    private double increment;

    public void runOpMode() throws InterruptedException{

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        waitForStart();
        while (opModeIsActive() && !isStopRequested()){
            // Increment controls
            if(gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
                increment+=0.1;
                if(increment > 1) increment=1;
            } else if(gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
                increment-=0.1;
                if(increment < -1) increment=-1;
            }

            //Hardware controls
            if(gamepadEx.isDown(GamepadKeys.Button.Y)) rightFront.setPower(increment);
            else rightFront.setPower(0);
            if(gamepadEx.isDown(GamepadKeys.Button.X)) leftFront.setPower(increment);
            else leftFront.setPower(0);
            if(gamepadEx.isDown(GamepadKeys.Button.A)) leftBack.setPower(increment);
            else leftBack.setPower(0);
            if(gamepadEx.isDown(GamepadKeys.Button.B)) rightBack.setPower(increment);
            else rightBack.setPower(0);

            gamepadEx.readButtons();

            telemetry.addData("leftFront: ", leftBack.getPower());
            telemetry.addData("leftBack: ", leftBack.getPower());
            telemetry.addData("rightFront: ", rightFront.getPower());
            telemetry.addData("rightBack: ", rightBack.getPower());
            telemetry.update();
        }
    }
}
