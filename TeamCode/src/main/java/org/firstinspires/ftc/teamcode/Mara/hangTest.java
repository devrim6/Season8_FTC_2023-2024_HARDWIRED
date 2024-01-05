package org.firstinspires.ftc.teamcode.Mara;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.checkerframework.checker.units.qual.A;

@TeleOp (name="hangTest",group = "testing")
public class hangTest extends LinearOpMode {
    DcMotor hangMotor;
    @Override
    public void runOpMode() throws InterruptedException {
        hangMotor=hardwareMap.get(DcMotor.class,"hangMotor");
        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.b){
                hangMotor.setPower(0.5);
            }else{
                hangMotor.setPower(0);
            }
            if(gamepad1.a){
                hangMotor.setPower(-0.5);
            }else{
                hangMotor.setPower(0);
            }
        }
    }
}