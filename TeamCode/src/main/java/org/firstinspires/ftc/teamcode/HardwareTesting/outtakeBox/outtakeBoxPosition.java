package org.firstinspires.ftc.teamcode.HardwareTesting.outtakeBox;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Variables.DefVal;

@TeleOp(name="Test-Outtake-Positions", group = "testing")
public class outtakeBoxPosition extends LinearOpMode {
    public ServoEx outtakePitchLeft, outtakePitchRight, outtakeYaw, outtakeRollLeft, outtakeRollRight;

    @Override
    public void runOpMode() throws InterruptedException {
        boolean a=false,b=true,x=false,y=false,upper=false,bottom=false;
        GamepadEx gamepadEx=new GamepadEx(gamepad1);
        outtakePitchLeft = new SimpleServo(hardwareMap, "outtakePitchLeft", Math.toRadians(0), Math.toRadians(360));
        outtakePitchRight = new SimpleServo(hardwareMap, "outtakePitchRight", Math.toRadians(0), Math.toRadians(360));
        outtakeYaw = new SimpleServo(hardwareMap, "outtakeYaw", Math.toRadians(0), Math.toRadians(360));
        outtakeRollLeft = new SimpleServo(hardwareMap, "outtakeRollLeft", Math.toRadians(0), Math.toRadians(360));
        outtakeRollRight = new SimpleServo(hardwareMap, "outtakeRollRight", Math.toRadians(0), Math.toRadians(360));

        outtakePitchLeft.setInverted(true);
        outtakeRollLeft.setInverted(true);
        outtakeYaw.setInverted(true);

        // Outtake arms
        outtakePitchRight.turnToAngle(DefVal.pivot0);
        outtakePitchLeft.turnToAngle(DefVal.pivot0);

        //outtakeYaw.turnToAngle(DefVal.yaw0);

        // Outtake box
        outtakeRollLeft.turnToAngle(DefVal.roll0);
        outtakeRollRight.turnToAngle(DefVal.roll0);

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
                    //outtakeYaw.turnToAngle(DefVal.yaw0);
                } //else outtakeYaw.turnToAngle(DefVal.yaw90);
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

            gamepadEx.readButtons();
            telemetry.addData("pivot: ", a);
            telemetry.addData("yaw: ", b);
            telemetry.addData("roll: ", x);
            telemetry.addData("latch: ", y);
            telemetry.addData("uClaw: ", upper);
            telemetry.addData("bClaw: ", bottom);
            telemetry.addData("servoPitchLeft: ", outtakePitchLeft.getAngle());
            telemetry.addData("servoPitchRight: ", outtakePitchRight.getAngle());
            telemetry.addData("yaw: ", outtakeYaw.getAngle());
            telemetry.addData("servoRollLeft: ", outtakeRollLeft.getAngle());
            telemetry.addData("servoRollRight: ", outtakeRollLeft.getAngle());
            telemetry.update();
        }
    }
}
