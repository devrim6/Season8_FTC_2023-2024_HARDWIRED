package org.firstinspires.ftc.teamcode.HardwareTesting;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp (name="gianiTest")
public class giani extends LinearOpMode {
    DcMotor glisieraST;
    @Override 
    public void runOpMode() throws InterruptedException {
        glisieraST=hardwareMap.get(DcMotor.class,"glisieraST");
        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.a){
                glisieraST.setPower(0.4);
            } else {
                glisieraST.setPower(0);
            }
        }
    }
}
