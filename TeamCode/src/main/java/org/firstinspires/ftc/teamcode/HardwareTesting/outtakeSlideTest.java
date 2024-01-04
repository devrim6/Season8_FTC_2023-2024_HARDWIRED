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

    double PI = Math.PI;
    double GEAR_RATIO = 3*5;
    double WHEEL_DIAMETER_CM = 3.565;
    double GEAR_MOTOR_REV_ULTRAPLANETARY = 28 * GEAR_RATIO;
    final double TICKS_PER_CM_Z = GEAR_MOTOR_REV_ULTRAPLANETARY / (WHEEL_DIAMETER_CM * PI);

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

        slideMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        GamepadEx gamepadEx = new GamepadEx(gamepad2);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()){
            boolean isOCLeft = slideMotorLeft.isOverCurrent(), isOCRight = slideMotorRight.isOverCurrent();

            if(gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                runToPosition(DIRECTION.LOW);
            }
            if(gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) runToPosition(DIRECTION.GROUND);
            if(gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_UP)) runToPosition(DIRECTION.HIGH);
            if(gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) runToPosition(DIRECTION.MIDDLE);

            if(isOCLeft || isOCRight){
                slideMotorLeft.setPower(0);
                slideMotorRight.setPower(0);
            } else if(gamepadEx.isDown(GamepadKeys.Button.A)) {
                slideMotorRight.setPower(0.4);
                slideMotorLeft.setPower(0.4);
            } else if(gamepadEx.isDown(GamepadKeys.Button.B)){
                slideMotorRight.setPower(-0.4);
                slideMotorLeft.setPower(-0.4);
            }
            else {
                slideMotorLeft.setPower(0);
                slideMotorRight.setPower(0);
            }


            gamepadEx.readButtons();
            telemetry.addData("direction: ", dir.toString());
            telemetry.addData("slideLeft: ", slideMotorLeft.getCurrentPosition());
            telemetry.addData("slideRight: ", slideMotorRight.getCurrentPosition());
            telemetry.addData("s pos", slideMotorLeft.getTargetPosition());
            telemetry.addData("r pos", slideMotorRight.getTargetPosition());
            telemetry.addData("isOCLeft: ",isOCLeft);
            telemetry.addData("isOCRight: ", isOCRight);
            telemetry.update();
        }
    }
    public void runToPosition(DIRECTION direction){
        dir = direction;
        slideMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        switch(direction){
            case HIGH:
                slideMotorLeft.setTargetPosition((int)(DefVal.LiftHIGH*TICKS_PER_CM_Z));
                slideMotorRight.setTargetPosition((int)(DefVal.LiftHIGH*TICKS_PER_CM_Z));
                break;
            case MIDDLE:
                slideMotorLeft.setTargetPosition((int)(DefVal.LiftMIDDLE*TICKS_PER_CM_Z));
                slideMotorRight.setTargetPosition((int)(DefVal.LiftMIDDLE*TICKS_PER_CM_Z));
                break;
            case LOW:
                slideMotorLeft.setTargetPosition((int)(DefVal.LiftLOW*TICKS_PER_CM_Z));
                slideMotorRight.setTargetPosition((int)(DefVal.LiftLOW*TICKS_PER_CM_Z));
                break;
            case GROUND:
                slideMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                slideMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                slideMotorLeft.setTargetPosition((int)(DefVal.LiftGROUND*TICKS_PER_CM_Z));
                slideMotorRight.setTargetPosition((int)(DefVal.LiftGROUND*TICKS_PER_CM_Z));
                break;
        }
        slideMotorRight.setPower(0.8);
        slideMotorLeft.setPower(0.8);
    }
}
