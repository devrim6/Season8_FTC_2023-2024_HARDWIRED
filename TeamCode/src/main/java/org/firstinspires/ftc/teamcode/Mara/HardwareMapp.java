package org.firstinspires.ftc.teamcode.Mara;

import android.graphics.Color;

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
import org.opencv.core.Scalar;

public class HardwareMapp {

    public enum LEDColor{
        Purple,
        Green,
        Yellow,
        White,
        None
    }
    public DcMotorEx hangMotor;  //motor pentru ridicat robotul in hang
    public DcMotorEx misumMotorLeft;  //motor pentru misum-ul stang
    public DcMotorEx misumMotorRight;  //motor pentru misum-ul drept
    public DcMotorEx intakeMotor;  //motor pentru maturice           //motoare

    public CRServo intakeServo;  //servo CR pentru intake
    public Servo outakeServo;  //servo pentru deschis outake-ul
    public Servo planeServo; //servo pentru avion               //servo-uri

    public SensorColor SensorfirstHook;  //senzor pentru primul pixel
    public SensorColor SensorsecondHook;  //senzor pentru al doilea pixel

    public LEDColor LEDdowngreen;
    public LEDColor LEDupgreen;
    public LEDColor LEDdownred;
    public LEDColor LEDupred;

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

        SensorfirstHook=HW.get(SensorColor.class,"firstHookPixel");
        SensorsecondHook=HW.get(SensorColor.class,"secondHookPixel");

        LEDdowngreen=HW.get(LEDColor.class,"LEDdownGreen");
        LEDdownred=HW.get(LEDColor.class,"LEDdownRed");
        LEDupgreen=HW.get(LEDColor.class,"LEDupGreen");
        LEDupred=HW.get(LEDColor.class,"LEDupRed");

        //imu=HW.get(BNO055IMU.class,"imu");
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
    float[] hsvValues = new float[3];
    Scalar white=new Scalar(0,0,100);
    Scalar green=new Scalar(126,88,74);
    Scalar purple=new Scalar(280,39,79);
    Scalar yellow=new Scalar(48,91,99);
    Scalar detectedColorHSV = new Scalar(hsvValues[0], hsvValues[1], hsvValues[2]);

    public Action pixelColor(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                //Color.RGBToHSV(colorSensor.red(), colorSensor.green(), colorSensor.blue(), hsvValues);
                if(detectedColorHSV==white){
                    return white.isReal();
                }
                if(detectedColorHSV==green){
                    return green.isReal();
                }
                if(detectedColorHSV==purple){
                    return purple.isReal();
                }
                if(detectedColorHSV==yellow){
                    return yellow.isReal();
                }
                return false;
            }
        };
    }
    public Action LedColor(){  //nu merge inca
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (detectedColorHSV.equals(green)) {
                    //return LEDColor.Green;
                }
                if (detectedColorHSV.equals(yellow)) {
                    //return LEDColor.Yellow;
                }
                if (detectedColorHSV.equals(white)) {
                    //return LEDColor.White;
                }
                if (detectedColorHSV.equals(purple)) {
                    //return LEDColor.Purple;
                }
                return false;
            }
        };
    }
    /*public class imu{
        public double TILT_THRESHOLD = 20;
        public double ACCEL_THRESHOLD = 20;  //am pus valori cam random. Trebuie sa ma documentez
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        Acceleration accel = imu.getLinearAcceleration();

        double rollAngle = angles.secondAngle;
        boolean isTilted = Math.abs(rollAngle) > TILT_THRESHOLD;

        double lateralAcceleration = accel.yAccel;
        boolean hasLateralAcceleration = Math.abs(lateralAcceleration) > ACCEL_THRESHOLD;

    }*/
}