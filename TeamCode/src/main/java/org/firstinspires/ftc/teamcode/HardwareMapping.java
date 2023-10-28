package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.units.qual.A;

public class HardwareMapping {
    private double pivotAngle;

    double PI = 3.1415;
    double GEAR_MOTOR_GOBILDA_312_TICKS = 537.7;
    double WHEEL_DIAMETER_CM = 3.565;
    double TICKS_PER_CM_Z = GEAR_MOTOR_GOBILDA_312_TICKS / (WHEEL_DIAMETER_CM * PI);

    public static Servo intakeServoLeft, intakeServoRight, outtakePitchLeft, outtakePitchRight, outtakeYaw, outtakeRollLeft, outtakeRollRight,
                 outtakeLatch, outtakeClawUpper, outtakeClawBottom;
    public static CRServo intakeServoRoller;
    public static DcMotorEx intakeMotor;
    public DcMotorEx hangMotor;
    public DcMotorEx slideMotorLeft;
    public DcMotorEx slideMotorRight;
    HardwareMap hwMap = null;
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
    }

    public void resetEncoder(){
        hangMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
    }
}
