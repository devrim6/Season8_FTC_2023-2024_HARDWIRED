package org.firstinspires.ftc.teamcode.Mara;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="TestMotorServo")
public class TestMotorServo extends LinearOpMode {

    DcMotor motor;
    CRServo servo;

    @Override
    public void runOpMode() throws InterruptedException {
        motor=hardwareMap.get(DcMotor.class,"motor");
        servo=hardwareMap.get(CRServo.class,"servo");

        waitForStart();

        while(opModeIsActive()){
            motor.setPower(gamepad1.right_trigger);
            servo.setPower(0.7);

            if(gamepad1.a){
                servo.setPower(1);
            }else{
                servo.setPower(0);
            }

            if(gamepad1.x){
                servo.setPower(1);
                motor.setPower(0.5);
            }else{
                servo.setPower(0);
                motor.setPower(0);
            }
            telemetry.addData("Servo Power", servo.getPower());
            telemetry.addData("Motor Power", motor.getPower());
            telemetry.update();
        }
    }
}
