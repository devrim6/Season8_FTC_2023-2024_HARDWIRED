package org.firstinspires.ftc.teamcode.Autonomous.Red.PixelParking;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Variables.DefVal;

@Autonomous(name="Autoparcare")
public class Autoparcare extends LinearOpMode {
    public Servo intakeServoLeft, intakeServoRight;
    public DcMotorEx leftFront, leftBack, rightBack, rightFront;
    @Override
    public void runOpMode() throws InterruptedException {
        intakeServoLeft=hardwareMap.get(Servo.class,"intakeServoLeft");
        intakeServoRight=hardwareMap.get(Servo.class,"intakeServoRight");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeServoRight.setPosition(DefVal.iLevel6);
        intakeServoLeft.setPosition(DefVal.iLevel6);
        waitForStart();
        while(opModeIsActive() && !isStopRequested()){
            intakeServoRight.setPosition(DefVal.iLevel6);
            intakeServoLeft.setPosition(DefVal.iLevel6);
            new SleepAction(3);
            leftFront.setPower(0.5);
            leftBack.setPower(0.5);
            rightBack.setPower(0.5);
            rightFront.setPower(0.5);
           new  SleepAction(1);
            leftFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);
            rightFront.setPower(0);
           new SleepAction(6);
        }
    }
}
