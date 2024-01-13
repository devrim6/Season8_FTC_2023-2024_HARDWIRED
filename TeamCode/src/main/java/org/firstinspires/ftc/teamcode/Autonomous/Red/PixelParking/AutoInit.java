package org.firstinspires.ftc.teamcode.Autonomous.Red.PixelParking;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Variables.DefVal;

@Autonomous(name="AutoInit")
public class AutoInit extends LinearOpMode {
    public Servo intakeServoLeft, intakeServoRight;

    @Override
    public void runOpMode() throws InterruptedException {
        /*intakeServoLeft=hardwareMap.get(Servo.class,"intakeServoLeft");
        intakeServoRight=hardwareMap.get(Servo.class,"intakeServoRight");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");*/
        intakeServoRight.setPosition(DefVal.iLevel6);
        intakeServoLeft.setPosition(DefVal.iLevel6);
        waitForStart();
        while(opModeIsActive() && !isStopRequested()){
            intakeServoRight.setPosition(DefVal.iLevel6);
            intakeServoLeft.setPosition(DefVal.iLevel6);
            /*Thread.sleep(20000);
            leftFront.setPower(0.5);
            leftBack.setPower(-0.5);
            rightBack.setPower(0.5);
            rightFront.setPower(-0.5);
            sleep(7000);
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);
            rightFront.setPower(0);
            sleep(5000);*/
        }
    }
}
