package org.firstinspires.ftc.teamcode.Mara;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="TestMotoareMara")
public class TestMotoare extends LinearOpMode {
    public DcMotor motorLeft;
    public DcMotor motorRight;
    @Override
    public void runOpMode() throws InterruptedException {
        motorLeft=hardwareMap.get(DcMotor.class,"slideMotorLeft");
        motorRight=hardwareMap.get(DcMotor.class,"slideMotorRight");

        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.a){
                motorLeft.setPower(1);
            }
            if(gamepad1.b){
                motorLeft.setPower(-1);
            }
            if(gamepad1.x){
                motorRight.setPower(1);
            }
            if(gamepad1.y){
                motorRight.setPower(-1);
            }
        }
    }
}
