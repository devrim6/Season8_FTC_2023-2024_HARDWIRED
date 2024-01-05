package org.firstinspires.ftc.teamcode.Mara;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="planeTest",group = "testing")
public class planeTest extends LinearOpMode {
    Servo planeServo;
    @Override
    public void runOpMode() throws InterruptedException {
        planeServo=hardwareMap.get(Servo.class,"planeServo");
        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.a){
                planeServo.setPosition(0.2);
            }
            telemetry.addData("ServoPosition ",planeServo.getPosition());
            telemetry.update();
        }
    }
}
