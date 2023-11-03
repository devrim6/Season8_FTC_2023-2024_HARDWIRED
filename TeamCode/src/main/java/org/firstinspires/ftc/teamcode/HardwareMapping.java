package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.arcrobotics.ftclib.hardware.SensorColor;

import org.checkerframework.checker.units.qual.A;

public class HardwareMapping {
    private double pivotAngle;

    double PI = 3.1415;
    double GEAR_MOTOR_GOBILDA_312_TICKS = 537.7;
    double WHEEL_DIAMETER_CM = 3.565;
    double TICKS_PER_CM_Z = GEAR_MOTOR_GOBILDA_312_TICKS / (WHEEL_DIAMETER_CM * PI);

    public Servo intakeServoLeft, intakeServoRight, outtakePitchLeft, outtakePitchRight, outtakeYaw, outtakeRollLeft, outtakeRollRight,
                 outtakeLatch, outtakeClawUpper, outtakeClawBottom;
    public CRServo intakeServoRoller;
    public DcMotorEx intakeMotor;
    public DcMotorEx hangMotor;
    public DcMotorEx slideMotorLeft;
    public DcMotorEx slideMotorRight;

    public GamepadEx gamepad1Ex, gamepad2Ex;

    public SensorColor bottomHookSensor, upperHookSensor;
    HardwareMap hwMap = null;
    public HardwareMapping(){}
    public void init(HardwareMap ahwMap){
        hwMap = ahwMap;

        /* Motoare baza sunt in MecanumDrive */
        /* Servos */
        intakeServoLeft = hwMap.get(Servo.class, "intakeServoLeft");
        intakeServoRight = hwMap.get(Servo.class, "intakeServoRight");
        intakeServoRoller = hwMap.get(CRServo.class, "intakeServoRoller");
        outtakePitchLeft = hwMap.get(Servo.class, "outtakePitchLeft");
        outtakePitchRight = hwMap.get(Servo.class, "outtakePitchRight");
        outtakeYaw = hwMap.get(Servo.class, "outtakeYaw");
        outtakeRollLeft = hwMap.get(Servo.class, "outtakeRollLeft");
        outtakeRollRight = hwMap.get(Servo.class, "outtakeRollRight");
        outtakeLatch = hwMap.get(Servo.class, "outtakeLatch");
        outtakeClawBottom = hwMap.get(Servo.class, "outtakeClawBottom");
        outtakeClawUpper = hwMap.get(Servo.class, "outtakeClawUpper");

        /* Motors */
        intakeMotor = hwMap.get(DcMotorEx.class, "intakeMotor");
        hangMotor = hwMap.get(DcMotorEx.class, "hangMotor");
        slideMotorLeft = hwMap.get(DcMotorEx.class, "slideMotorLeft");
        slideMotorRight = hwMap.get(DcMotorEx.class, "slideMotorRight");

        slideMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hangMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /* Sensors */
        upperHookSensor = ahwMap.get(SensorColor.class, "upperHookSensor");
        bottomHookSensor = ahwMap.get(SensorColor.class, "bottomHookSensor");

    }

    public void gamepadInit(Gamepad gmpd1, Gamepad gmpd2){
        gamepad1Ex = new GamepadEx(gmpd1);
        gamepad2Ex = new GamepadEx(gmpd2);
    }

    public void resetEncoder(){
        hangMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public Object[] getColorSensorHSV(){
        float[] hsv1= new float[3];
        float[] hsv2= new float[3];
        upperHookSensor.RGBtoHSV(upperHookSensor.red(), upperHookSensor.green(), upperHookSensor.blue(), hsv1);
        bottomHookSensor.RGBtoHSV(bottomHookSensor.red(), bottomHookSensor.green(), bottomHookSensor.blue(), hsv2);
        return new Object[]{hsv1, hsv2};
    }
    public String checkColorRange(String sensor){
        float[] hsv = new float[3];
        switch (sensor){
            case "upper":
                hsv = (float[])getColorSensorHSV()[0];
                break;
            case "bottom":
                hsv = (float[])getColorSensorHSV()[1];
                break;
        }
        if(hsv[0] <= 320 && hsv[0] >= 280) return "purple";
        else if(hsv[0] <= 130 && hsv[0] >= 110) return "green";
        else if(hsv[0] <= 74 && hsv[0] >= 51) return "yellow";
        else if(hsv[1] <= 10 && hsv[1] >= 0) return "white";
        return "none";
    }

    public class Outtake {
        public Outtake() {} // The constructor

        public Action pivot(double angle, double angle2){
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    outtakePitchLeft.setPosition(angle);
                    outtakePitchRight.setPosition(angle2);
                    return false;
                }
            };}
        public Action yaw(double angle){
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    outtakeYaw.setPosition(angle);
                    return false;
                }
            };}
        public Action roll(double angle, double angle2){
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    outtakeRollLeft.setPosition(angle);
                    outtakeRollRight.setPosition(angle2);
                    return false;
                }
            };}
        public Action latch(String state){
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    switch(state){
                        case "open":
                            outtakeLatch.setPosition(0.5);
                            break;
                        case "closed" :
                            outtakeLatch.setPosition(0);
                            break;
                    }
                    return false;
                }
            };}
            public Action upperHook(String state){
                return new Action() {
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        switch(state){
                            case "open":
                                outtakeClawUpper.setPosition(0.5);
                                break;
                            case "closed" :
                                outtakeClawUpper.setPosition(0);
                                break;
                        }
                        return false;}
                };}
            public Action bottomHook(String state){
                return new Action() {
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        switch(state){
                            case "open":
                                outtakeClawBottom.setPosition(0.5);
                                break;
                            case "closed" :
                                outtakeClawBottom.setPosition(0);
                                break;
                        }
                        return false;}
                };}
        public Action runToPosition(String direction){
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    switch(direction){
                        case "third":
                            slideMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            slideMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            slideMotorLeft.setTargetPosition((int)(24*TICKS_PER_CM_Z));
                            slideMotorRight.setTargetPosition(-(int)(24*TICKS_PER_CM_Z));
                            slideMotorRight.setPower(1);
                            slideMotorLeft.setPower(1);
                        case "second":
                            slideMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            slideMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            slideMotorLeft.setTargetPosition((int)(10*TICKS_PER_CM_Z));
                            slideMotorRight.setTargetPosition(-(int)(10*TICKS_PER_CM_Z));
                            slideMotorRight.setPower(1);
                            slideMotorLeft.setPower(1);
                        case "first":
                            slideMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            slideMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            slideMotorLeft.setTargetPosition((int)(5*TICKS_PER_CM_Z));
                            slideMotorRight.setTargetPosition(-(int)(5*TICKS_PER_CM_Z));
                            slideMotorRight.setPower(1);
                            slideMotorLeft.setPower(1);
                        case "ground":
                            slideMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            slideMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            slideMotorLeft.setTargetPosition((int)(0*TICKS_PER_CM_Z));
                            slideMotorRight.setTargetPosition(-(int)(0*TICKS_PER_CM_Z));
                            slideMotorRight.setPower(1);
                            slideMotorLeft.setPower(1);
                    }
                    return false;
                }
            };}
    }

    public class Intake {
        public Intake() {} // The constructor

        public Action powerOn(){
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    intakeMotor.setPower(0.5);
                    intakeServoRoller.setPower(-0.3);
                    return false;
                }
            };}
        public Action reverse(){
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    intakeMotor.setPower(-0.5);
                    intakeServoRoller.setPower(0.3);
                    return false;
                }
            };}
        public Action stop(){
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    intakeMotor.setPower(0);
                    intakeServoRoller.setPower(0);
                    return false;
                }
            };}
        public Action angle(int level){
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    switch (level){
                        case 1:
                            intakeServoLeft.setPosition(0);
                            intakeServoRight.setPosition(0);
                            break;
                        case 2:
                            intakeServoLeft.setPosition(0.2);
                            intakeServoRight.setPosition(-0.2);
                            break;
                        case 3:
                            intakeServoLeft.setPosition(0.4);
                            intakeServoRight.setPosition(-0.4);
                            break;
                        case 4:
                            intakeServoLeft.setPosition(0.6);
                            intakeServoRight.setPosition(-0.6);
                            break;
                        case 5:
                            intakeServoLeft.setPosition(0.8);
                            intakeServoRight.setPosition(-0.8);
                            break;
                    }
                    return false;
                }
            };}
    }
}
