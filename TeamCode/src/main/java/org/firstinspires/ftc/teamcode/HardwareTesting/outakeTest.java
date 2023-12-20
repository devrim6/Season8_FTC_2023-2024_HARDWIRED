package org.firstinspires.ftc.teamcode.HardwareTesting;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="outtakeTest")
public class outakeTest extends LinearOpMode {
    /**
     * Override this method and place your code here.
     * <p>
     * Please do not catch {@link InterruptedException}s that are thrown in your OpMode
     * unless you are doing it to perform some brief cleanup, in which case you must exit
     * immediately afterward. Once the OpMode has been told to stop, your ability to
     * control hardware will be limited.
     *
     * @throws InterruptedException When the OpMode is stopped while calling a method
     *                              that can throw {@link InterruptedException}
     */

    public ServoEx outtakePitchLeft, outtakePitchRight, outtakeYaw, outtakeRollLeft, outtakeRollRight;
    public Servo outtakeLatch, outtakeClawUpper, outtakeClawBottom;
    private GamepadEx gamepadEx=new GamepadEx(gamepad1);
    double increment1,increment2;

    @Override
    public void runOpMode() throws InterruptedException {
        outtakePitchLeft=hardwareMap.get(ServoEx.class,"outtakePitchLeft");
        outtakePitchRight=hardwareMap.get(ServoEx.class,"outtakePitchRight");
        outtakeYaw=hardwareMap.get(ServoEx.class,"outtakeYaw");
        outtakeRollLeft=hardwareMap.get(ServoEx.class,"outtakeRollLeft");
        outtakeRollRight=hardwareMap.get(ServoEx.class,"outtakeRollRight");
        outtakeLatch=hardwareMap.get(Servo.class,"outtakeLatch");
        outtakeClawBottom=hardwareMap.get(Servo.class,"outtakeClawBottom");
        outtakeClawUpper=hardwareMap.get(Servo.class,"outtakeClawUpper");

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){

            if(gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
                increment1+=0.1;
                if(increment1>1) increment1=1;
            }
            if(gamepadEx.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
                increment1-=0.1;
                if(increment1<-0.1) increment1=0.1;
            }

            if(gamepadEx.wasJustPressed(GamepadKeys.Button.A)) outtakePitchLeft.rotateByAngle(increment1);
            if(gamepadEx.wasJustPressed(GamepadKeys.Button.A)) outtakePitchRight.rotateByAngle(increment1);
            if(gamepadEx.wasJustPressed(GamepadKeys.Button.B)) outtakeRollLeft.rotateByAngle(increment1);
            if(gamepadEx.wasJustPressed(GamepadKeys.Button.B)) outtakeRollRight.rotateByAngle(increment1);
            if(gamepadEx.wasJustPressed(GamepadKeys.Button.X)) outtakeClawBottom.setPosition(increment1);
            if(gamepadEx.wasJustPressed(GamepadKeys.Button.X)) outtakeClawUpper.setPosition(increment1);
            if(gamepadEx.wasJustPressed(GamepadKeys.Button.Y)) outtakeLatch.setPosition(increment1);
            if(gamepadEx.wasJustPressed(GamepadKeys.Button.Y)) outtakeYaw.rotateByAngle(increment1);

        }

    }
}
