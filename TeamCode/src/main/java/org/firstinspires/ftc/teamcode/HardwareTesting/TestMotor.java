package org.firstinspires.ftc.teamcode.HardwareTesting;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="motorTest")
public class TestMotor extends LinearOpMode {
    DcMotor motor;
    @Override
    public void runOpMode() throws InterruptedException {
        motor=hardwareMap.get(DcMotor.class,"motor");
        waitForStart();
        while(opModeIsActive())
        {
            if(gamepad1.a)
            {
                motor.setPower(0.4);
            }else {
                motor.setPower(0);
            }
        }
    }
}
