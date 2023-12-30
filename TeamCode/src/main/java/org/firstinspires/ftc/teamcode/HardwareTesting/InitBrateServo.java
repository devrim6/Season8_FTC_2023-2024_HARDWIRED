package org.firstinspires.ftc.teamcode.HardwareTesting;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name = "TESTT INIT SERVO BRATE")
public class InitBrateServo extends LinearOpMode {
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
    @Override
    public void runOpMode() throws InterruptedException {
//        ServoEx outtakePitchLeft = new SimpleServo(hardwareMap, "outtakePitchLeft", Math.toRadians(0), Math.toRadians(350));
//        ServoEx outtakePitchRight = new SimpleServo(hardwareMap, "outtakePitchRight", Math.toRadians(0), Math.toRadians(350));
        ServoEx outtakeRollLeft = new SimpleServo(hardwareMap, "outtakeRollLeft", Math.toRadians(0), Math.toRadians(350));
        ServoEx outtakeRollRight = new SimpleServo(hardwareMap, "outtakeRollRight", Math.toRadians(0), Math.toRadians(350));
//        outtakePitchRight.turnToAngle(0);
//        outtakePitchLeft.turnToAngle(0);
        outtakeRollLeft.turnToAngle(0);
        outtakeRollRight.turnToAngle(0);
//        outtakePitchLeft.setInverted(true);
        outtakeRollLeft.setInverted(true);
        waitForStart();
        while (opModeIsActive() && !isStopRequested()){
            telemetry.update();
        }
    }
}
