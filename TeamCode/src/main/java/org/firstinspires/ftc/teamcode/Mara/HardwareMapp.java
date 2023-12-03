package org.firstinspires.ftc.teamcode.Mara;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorColor;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class HardwareMapp {

    /*public enum LEDColor{
        Purple,
        Green,
        Yellow,
        White,
        None
    }*/
    public DcMotorEx hangMotor;  //motor pentru rificat robotul in hang
    public DcMotorEx misumMotorLeft;  //motor pentru misum-ul stang
    public DcMotorEx misumMotorRight;  //motor pentru misum-ul drept
    public DcMotorEx intakeMotor;  //motor pentru maturice           //motoare

    public CRServo intakeServo;  //servo CR pentru intake
    public Servo outakeServo;  //servo pentru deschis outake-ul
    public Servo planeServo; //servo pentru avion               //servo-uri

    public SensorColor firstHook;  //senzor pentru primul pixel
    public SensorColor secondHook;  //senzor pentru al doilea pixel

    //public LEDColor LEDdowngreen;
    //public LEDColor LEDupgreen;
    //public LEDColor LEDdownred;
    //public LEDColor LEDupred;

    BNO055IMU imu;
    HardwareMap HW=null;
    public HardwareMapp(){}

    public void init() {

        //HW=hw;

        hangMotor=HW.get(DcMotorEx.class,"hangMotor");
        misumMotorLeft=HW.get(DcMotorEx.class,"misumMotorLeft");
        misumMotorRight=HW.get(DcMotorEx.class,"misumMotorRight");
        intakeMotor=HW.get(DcMotorEx.class,"intakeMotor");

        intakeServo=HW.get(CRServo.class,"intakeServo");
        outakeServo=HW.get(Servo.class,"outakeServo");
        planeServo=HW.get(Servo.class,"planeServo");

        firstHook=HW.get(SensorColor.class,"firstHookPixel");
        secondHook=HW.get(SensorColor.class,"secondHookPixel");

        /*LEDdowngreen=HW.get(LEDColor.class,"LEDdownGreen");
        LEDdownred=HW.get(LEDColor.class,"LEDdownRed");
        LEDupgreen=HW.get(LEDColor.class,"LEDupGreen");
        LEDupred=HW.get(LEDColor.class,"LEDupRed");*/

        imu=HW.get(BNO055IMU.class,"imu");
    }

    public Action launchPlane(){     //actiune pentru decolarea avionului
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                planeServo.setPosition(0.2);
                return false;
            }
        };
    }
    public Action Intake(String stare){     //actiune pentru intake (roller)
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                switch (stare){
                    case "lessThan2Pixels":
                        intakeServo.setPower(1);
                        break;
                    case "moreThan2Pixels":
                        intakeServo.setPower(-1);
                        break;
                }
                return false;
            }
        };
    }

    public Action hang(String stare){     //actiune pentru hang
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                switch (stare){
                    case "up":
                        hangMotor.setPower(1);
                        CommandScheduler.getInstance().schedule(new WaitCommand(1500));
                        hangMotor.setPower(0);
                        break;
                    case "hang":
                        hangMotor.setPower(0);
                        break;
                }
                return false;
            }
        };
    }

    public Action openOutake(String stare){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                switch (stare){
                    case "open":
                        outakeServo.setPosition(0.6);
                        break;
                    case "close":
                        outakeServo.setPosition(0);
                        break;
                }
                return false;
            }
        };
    }

    public Action maturice(String stare){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                switch (stare) {
                    case "lessThan2Pixels":
                        intakeMotor.setPower(1);
                        break;
                    case "moreThan2Pixels":
                        intakeMotor.setPower(-1);
                        break;
                }
                return false;
            }
        };
    }

    /*public Action pixelColor(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                return false;
            }
        };
    }*/
    public class imu{
        public double TILT_THRESHOLD = 20;
        public double ACCEL_THRESHOLD = 20;  //am pus valori cam random. Trebuie sa ma documentez
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        Acceleration accel = imu.getLinearAcceleration();

        double rollAngle = angles.secondAngle;
        boolean isTilted = Math.abs(rollAngle) > TILT_THRESHOLD;

        double lateralAcceleration = accel.yAccel;
        boolean hasLateralAcceleration = Math.abs(lateralAcceleration) > ACCEL_THRESHOLD;

    }
}