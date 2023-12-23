package org.firstinspires.ftc.teamcode.HardwareTesting;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Variables.DefVal;

@TeleOp(name="outtakeTest")
public class outakeTest extends LinearOpMode {
    public ServoEx outtakePitchLeft, outtakePitchRight, outtakeYaw, outtakeRollLeft, outtakeRollRight;
    public Servo outtakeLatch, outtakeClawUpper, outtakeClawBottom;

    @Override
    public void runOpMode() throws InterruptedException {
        boolean a=false,b=false,x=false,y=false,upper=false,bottom=false;
        GamepadEx gamepadEx=new GamepadEx(gamepad1);
        outtakePitchLeft=hardwareMap.get(ServoEx.class,"outtakePitchLeft");
        outtakePitchRight=hardwareMap.get(ServoEx.class,"outtakePitchRight");
        outtakeYaw=hardwareMap.get(ServoEx.class,"outtakeYaw");
        outtakeRollLeft=hardwareMap.get(ServoEx.class,"outtakeRollLeft");
        outtakeRollRight=hardwareMap.get(ServoEx.class,"outtakeRollRight");
        outtakeLatch=hardwareMap.get(Servo.class,"outtakeLatch");
        outtakeClawBottom=hardwareMap.get(Servo.class,"outtakeClawBottom");
        outtakeClawUpper=hardwareMap.get(Servo.class,"outtakeClawUpper");

        // Outtake arms
        outtakePitchRight.turnToAngle(0);
        outtakePitchLeft.turnToAngle(0);

        outtakeYaw.turnToAngle(0);

        // Outtake box
        outtakeRollLeft.turnToAngle(0);
        outtakeRollRight.turnToAngle(0);

        outtakeLatch.setPosition(0);

        outtakeClawBottom.setPosition(0);
        outtakeClawUpper.setPosition(0);

        outtakePitchLeft.setInverted(true);
        outtakeRollLeft.setInverted(true);

        telemetry.setMsTransmissionInterval(50);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){

            if(gamepadEx.wasJustPressed(GamepadKeys.Button.A)){
                a=!a;
                if(a){
                    outtakePitchLeft.turnToAngle(DefVal.pivot0);
                    outtakePitchRight.turnToAngle(DefVal.pivot0);
                } else {
                    outtakePitchLeft.turnToAngle(DefVal.pivot60);
                    outtakePitchRight.turnToAngle(DefVal.pivot60);
                }
            }
            if(gamepadEx.wasJustPressed(GamepadKeys.Button.B)){
                b=!b;
                if(b){
                    outtakeYaw.turnToAngle(DefVal.yaw0);
                } else outtakeYaw.turnToAngle(DefVal.yaw90);
            }
            if(gamepadEx.wasJustPressed(GamepadKeys.Button.X)){
                x=!x;
                if(x){
                    outtakeRollLeft.turnToAngle(DefVal.roll0);
                    outtakeRollRight.turnToAngle(DefVal.roll0);
                } else {
                    outtakeRollLeft.turnToAngle(DefVal.roll60);
                    outtakeRollRight.turnToAngle(DefVal.roll60);
                }
            }
            if(gamepadEx.wasJustPressed(GamepadKeys.Button.Y)){
                y=!y;
                if(y) outtakeLatch.setPosition(DefVal.latchOpen);
                else outtakeLatch.setPosition(DefVal.latchClosed);
            }
            if(gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
                upper=!upper;
                if(upper) outtakeClawUpper.setPosition(DefVal.upperHookOpen);
                else outtakeClawUpper.setPosition(DefVal.upperHookClosed);
            }
            if(gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
                bottom=!bottom;
                if(bottom) outtakeClawBottom.setPosition(DefVal.bottomHookOpen);
                else outtakeClawBottom.setPosition(DefVal.bottomHookClosed);
            }

            gamepadEx.readButtons();
            telemetry.addData("pivot: ", a);
            telemetry.addData("yaw: ", b);
            telemetry.addData("roll: ", x);
            telemetry.addData("latch: ", y);
            telemetry.addData("uClaw: ", upper);
            telemetry.addData("bClaw: ", bottom);
            telemetry.update();
        }
    }
}
