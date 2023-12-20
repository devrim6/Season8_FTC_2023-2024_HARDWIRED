package org.firstinspires.ftc.teamcode.HardwareTesting;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="intakeTest")
public class intakeTest extends LinearOpMode {
    private Servo intakeServoLeft, intakeServoRight;
   // private CRServo intakeServoRoller;
    private DcMotorEx intakeMotor;

    private double increment;

    public void runOpMode() throws InterruptedException{

        intakeServoLeft = hardwareMap.get(Servo.class, "intakeServoLeft");
        intakeServoRight = hardwareMap.get(Servo.class, "intakeServoRight");
        //intakeServoRoller = hardwareMap.get(CRServo.class, "intakeServoRoller");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        GamepadEx gamepadEx = new GamepadEx(gamepad2);

        intakeServoRight.setPosition(0);
        intakeServoLeft.setPosition(0);

        intakeServoRight.setDirection(Servo.Direction.REVERSE);

        waitForStart();
        while(opModeIsActive() && !isStopRequested()){
            // Increment controls
            if(gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
                increment+=0.1;
                if(increment > 1) increment=1;
            } else if(gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
                increment-=0.1;
                if(increment < -1) increment=-1;
            }

            //Hardware controls
            if(gamepadEx.wasJustPressed(GamepadKeys.Button.X)) intakeServoLeft.setPosition(increment);
            if(gamepadEx.wasJustPressed(GamepadKeys.Button.B)) intakeServoRight.setPosition(increment);
            if(gamepadEx.wasJustPressed(GamepadKeys.Button.Y)) intakeMotor.setPower(increment);
            //if(gamepadEx.wasJustPressed(GamepadKeys.Button.A)) intakeServoRoller.setPower(increment);

            gamepadEx.readButtons();

            telemetry.addData("Increment: ", increment);
            telemetry.addData("ServoLeft: ", intakeServoLeft.getPosition());
            telemetry.addData("ServoRight: ", intakeServoRight.getPosition());
            telemetry.addData("Motor: ", intakeMotor.getPower());
           // telemetry.addData("ServoRoller: ", intakeServoRoller.getPower());
            telemetry.update();
        }
    }
}
