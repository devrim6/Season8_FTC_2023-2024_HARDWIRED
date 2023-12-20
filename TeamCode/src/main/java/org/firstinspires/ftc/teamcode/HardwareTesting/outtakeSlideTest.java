package org.firstinspires.ftc.teamcode.HardwareTesting;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Variables.DefVal;

@TeleOp(name="outtakeSlideTest")
public class outtakeSlideTest extends LinearOpMode {

    private DcMotorEx slideMotorLeft, slideMotorRight;

    private GamepadEx gamepadEx = new GamepadEx(gamepad2);

    double PI = 3.1415;
    double GEAR_MOTOR_GOBILDA_312_TICKS = 537.7;
    double WHEEL_DIAMETER_CM = 3.565;
    double TICKS_PER_CM_Z = GEAR_MOTOR_GOBILDA_312_TICKS / (WHEEL_DIAMETER_CM * PI);

    private enum DIRECTION {
        GROUND,
        LOW,
        MIDDLE,
        HIGH
    }

    DIRECTION dir = DIRECTION.GROUND;

    @Override
    public void runOpMode() throws InterruptedException {

        slideMotorLeft = hardwareMap.get(DcMotorEx.class, "slideMotorLeft");
        slideMotorRight = hardwareMap.get(DcMotorEx.class, "slideMotorRight");

        slideMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()){

            if(gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) runToPosition(DIRECTION.LOW);
            if(gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) runToPosition(DIRECTION.GROUND);
            if(gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_UP)) runToPosition(DIRECTION.HIGH);
            if(gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) runToPosition(DIRECTION.MIDDLE);

            gamepadEx.readButtons();
            telemetry.addData("direction: ", dir.toString());
            telemetry.addData("slideLeft: ", slideMotorLeft.getCurrentPosition());
            telemetry.addData("slideRight: ", slideMotorRight.getCurrentPosition());
            telemetry.update();
        }
    }
    private void runToPosition(DIRECTION direction){
        dir = direction;
        slideMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        switch(direction){
            case HIGH:
                slideMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                slideMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                slideMotorLeft.setTargetPosition((int)(DefVal.LiftHIGH*TICKS_PER_CM_Z));
                slideMotorRight.setTargetPosition((int)(DefVal.LiftHIGH*TICKS_PER_CM_Z));
            case MIDDLE:
                slideMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                slideMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                slideMotorLeft.setTargetPosition((int)(DefVal.LiftMIDDLE*TICKS_PER_CM_Z));
                slideMotorRight.setTargetPosition((int)(DefVal.LiftMIDDLE*TICKS_PER_CM_Z));
            case LOW:
                slideMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                slideMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                slideMotorLeft.setTargetPosition((int)(DefVal.LiftLOW*TICKS_PER_CM_Z));
                slideMotorRight.setTargetPosition((int)(DefVal.LiftLOW*TICKS_PER_CM_Z));
            case GROUND:
                slideMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                slideMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                slideMotorLeft.setTargetPosition((int)(DefVal.LiftGROUND*TICKS_PER_CM_Z));
                slideMotorRight.setTargetPosition((int)(DefVal.LiftGROUND*TICKS_PER_CM_Z));
        }
        slideMotorRight.setPower(1);
        slideMotorLeft.setPower(1);
    }
}
