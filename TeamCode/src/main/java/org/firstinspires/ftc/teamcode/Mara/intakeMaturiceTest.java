package org.firstinspires.ftc.teamcode.Mara;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name="intakeMaturiceTest",group = "testing")
public class intakeMaturiceTest extends LinearOpMode {
    DcMotor intakeMotor;
    DcMotor maturiceMotor;
    @Override
    public void runOpMode() throws InterruptedException {
        intakeMotor=hardwareMap.get(DcMotor.class,"intakeMotor");
        maturiceMotor=hardwareMap.get(DcMotor.class,"maturiceMotor");
        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.a){
                intakeMotor.setPower(0.5); maturiceMotor.setPower(0.5);
            } else {intakeMotor.setPower(0); maturiceMotor.setPower(0);}

            if(gamepad1.b){
                maturiceMotor.setPower(-0.5); intakeMotor.setPower(-0.5);
            } else {intakeMotor.setPower(0);maturiceMotor.setPower(0);}
            telemetry.addData("intakeMotorPower ",intakeMotor.getPower());
            telemetry.addData("maturiceMotorPower",maturiceMotor.getPower());
        }
    }
}
