package org.firstinspires.ftc.teamcode.Mara;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorColor;
import org.opencv.core.Scalar;

public class HardwareMapp {

    /*Scriu aici ce mai trebuie facut:
    * Ridicare coborare misumiuri-gata(mai trebuie pus tick per cm)
    * Prindere pixeli(hook)-gata(nush exact daca trebuie sa verific in ColorDetected)
    * Sa fac sa lumineze LED-urile-gata(mai trebuie blinking pentru alb)
    * Implementare senzori de culoare*/

    public enum LEDColor{
        Purple, //red
        Green, //green
        Yellow, //between red & green(amber)
        White, //blinking
        None //none
    }
    public DcMotorEx hangMotor;  //motor pentru ridicat robotul in hang
    public DcMotorEx misumMotorLeft;  //motor pentru misum-ul stang
    public DcMotorEx misumMotorRight;  //motor pentru misum-ul drept
    public DcMotorEx intakeMotor;  //motor pentru maturice           //motoare

    public CRServo intakeServo;  //servo CR pentru intake
    public Servo outakeServo;  //servo pentru deschis outake-ul
    public Servo planeServo; //servo pentru avion               //servo-uri

    public Servo servoHook1;
    public Servo servoHook2;

    public SensorColor SensorfirstHook;  //senzor pentru primul pixel
    public SensorColor SensorsecondHook;  //senzor pentru al doilea pixel

    public DigitalChannel LEDdowngreen;
    public DigitalChannel LEDupgreen;
    public DigitalChannel LEDdownred;
    public DigitalChannel LEDupred;

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
        servoHook1=HW.get(Servo.class,"hook1");
        servoHook2=HW.get(Servo.class,"hook2");

        SensorfirstHook=HW.get(SensorColor.class,"firstHookPixel");
        SensorsecondHook=HW.get(SensorColor.class,"secondHookPixel");

        LEDdowngreen=HW.get(DigitalChannel.class,"LEDdownGreen");
        LEDdownred=HW.get(DigitalChannel.class,"LEDdownRed");
        LEDupgreen=HW.get(DigitalChannel.class,"LEDupGreen");
        LEDupred=HW.get(DigitalChannel.class,"LEDupRed");

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
    Scalar detectedColorHSV = new Scalar(hsvValues[0], hsvValues[1], hsvValues[2]);

    public static class ColorRange{
        public Scalar[] greenColorRange = {
                new Scalar(40, 40, 40), // Valoare minimă HSV pentru verde
                new Scalar(80, 255, 255) // Valoare maximă HSV pentru verde
        };
        public Scalar[] yellowColorRange={
                new Scalar(20,100,100), //Valoare minima HSV pentru galben
                new Scalar(30,255,255) //Valoare maxima HSV pentru galben
        };
        public Scalar[] purpleColorRange={
                new Scalar(130,50,50), //Valoare minima HSV pentru mov
                new Scalar(160,255,255) //Valoare maxima HSV pentru mov
        };
        public Scalar[] whiteColorRange={
                new Scalar(0,0,200), //Valoare minima HSV pentru alb
                new Scalar(180,30,255) //Valoare maxima HSV pentru alb
        };
    }
    public LEDColor ColorDetected(Scalar targetColor, Scalar[] colorRange){
        ColorRange colorRangeDet=new ColorRange();
        if(detectedColorHSV.val[0] >= colorRangeDet.greenColorRange[0].val[0] && detectedColorHSV.val[0] <= colorRangeDet.greenColorRange[1].val[0]){
            //vede culoare verde
            //Leduri
            return LEDColor.Green;
            //cod pentru hook

        }
        if(detectedColorHSV.val[0] >= colorRangeDet.yellowColorRange[0].val[0] && detectedColorHSV.val[0] <= colorRangeDet.yellowColorRange[1].val[0]){
            //vede culoaregalben
            //Leduri
            return LEDColor.Yellow;
        }
        if(detectedColorHSV.val[0] >= colorRangeDet.whiteColorRange[0].val[0] && detectedColorHSV.val[0] <= colorRangeDet.whiteColorRange[1].val[0]){
            //vede culoare alb
            //Leduri
            return LEDColor.White;
        }
        if(detectedColorHSV.val[0] >= colorRangeDet.purpleColorRange[0].val[0] && detectedColorHSV.val[0] <= colorRangeDet.purpleColorRange[1].val[0]){
            //vede culoare mov
            //Leduri
            return LEDColor.Purple;
        }
        return LEDColor.None;
    }

    public Action LED(String led){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                DigitalChannel LED1=null;
                DigitalChannel LED2=null;
                if(led.equals("up")){
                    LED1=LEDupgreen;
                    LED2=LEDupred;
                }
                if(led.equals("down")){
                    LED1=LEDdowngreen;
                    LED2=LEDdownred;
                }
                return false;
            }
        };
    }

    public Action LEDforDrivers(String stare,DigitalChannel LED1,DigitalChannel LED2){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                switch (stare) {
                    case "NONE":
                        LED1.setState(false); //LED1=green,LED2=red
                        LED2.setState(false);
                    case "GREEN":
                        LED1.setState(true);
                        LED2.setState(false);
                    case "PURPLE":
                        LED1.setState(false);
                        LED2.setState(true);
                    case "YELLOW":
                        LED1.setState(true);
                        LED2.setState(true);
                    case "WHITE":
                        Actions.runBlocking(WhitePixelBlinking(LED1,LED2)); //trebuie facuta actiunea de blinking
                }
                return false;
            }
        };
    }

    public Action WhitePixelBlinking(DigitalChannel LED1, DigitalChannel LED2){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                //cod pentru bliking
                return false;
            }
        };
    }

    public Action misum(String stare){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                switch (stare){
                    case "GROUND":
                        misumMotorLeft.setPositionPIDFCoefficients(0);
                        misumMotorRight.setPositionPIDFCoefficients(0);
                    case "LOW":
                        misumMotorLeft.setPositionPIDFCoefficients(5);
                        misumMotorRight.setPositionPIDFCoefficients(5);
                    case "MIDDLE":
                        misumMotorLeft.setPositionPIDFCoefficients(10);
                        misumMotorRight.setPositionPIDFCoefficients(10);
                    case "HIGH":
                        misumMotorLeft.setPositionPIDFCoefficients(15);
                        misumMotorRight.setPositionPIDFCoefficients(15);
                }
                misumMotorLeft.setPower(1);
                misumMotorRight.setPower(1);
                return false;
            }
        };
    }

    public Action hook1(String stare){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                switch (stare){
                    case "pixel":
                        servoHook1.setPosition(0.6);
                    case "noPixel":
                        servoHook1.setPosition(0);
                }
                return false;
            }
        };
    }

    public Action hook2(String stare){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                switch (stare){
                    case "pixel":
                        servoHook2.setPosition(0.6);
                    case "noPixel":
                        servoHook2.setPosition(0);
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