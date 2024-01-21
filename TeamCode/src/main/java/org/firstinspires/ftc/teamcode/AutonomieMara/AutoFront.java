package org.firstinspires.ftc.teamcode.AutonomieMara;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Variables.DefVal;

@Autonomous(name = "AutoFront")
public class AutoFront extends LinearOpMode {

    public void runOpMode() throws InterruptedException{
        DcMotorEx frontLeft;
        DcMotorEx frontRight;
        DcMotorEx backLeft;
        DcMotorEx backRight;
        Servo intakeServoRight,intakeServoLeft;

        //TODO: change the time the robot moves for
         double forwardTime = 2.25;
          double forwardTime1=0.5;
        //TODO: change the power given to the wheels
          double power = 0.5;

        frontLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
        backLeft = hardwareMap.get(DcMotorEx.class, "leftBack");
        backRight = hardwareMap.get(DcMotorEx.class, "rightBack");
        intakeServoRight=hardwareMap.get(Servo.class,"intakeServoRight");
        intakeServoLeft=hardwareMap.get(Servo.class,"intakeServoLeft");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeServoLeft.setPosition(DefVal.iLevel6);
        intakeServoRight.setPosition(DefVal.iLevel6);

        ElapsedTime timer = new ElapsedTime();
        ElapsedTime timer1=new ElapsedTime();
        boolean timerStarted = false;
        boolean timer1Started=false;
        boolean timerstopped = false;
        waitForStart();
        while (opModeIsActive() && !isStopRequested()){
            if (!timerStarted) {
                timer.startTime();
                timer.reset();
                timerStarted = true;
            }
            if (timer.seconds() <= forwardTime) {
                frontLeft.setPower(-power);
                frontRight.setPower(-power);
                backLeft.setPower(-power);
                backRight.setPower(-power);
            } else {
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                timerstopped = true;
            }


            if(!timer1Started && timerstopped){
                timer1.startTime();
                timer1.reset();
                timer1Started=true;

            }
            if(timer1.seconds()<=forwardTime1){
                frontLeft.setPower(power);
                frontRight.setPower(power);
                backLeft.setPower(power);
                backRight.setPower(power);
            } else {
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
            }
        }
    }

}