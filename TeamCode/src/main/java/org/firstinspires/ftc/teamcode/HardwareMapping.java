package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.units.qual.A;

public class HardwareMapping {
    private double pivotAngle;

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
    }

    public static class Outtake {
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
    }

    public static class Intake {
        public Intake() {} // The constructor

        public Action power(double speed, double servoSpeed){
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    intakeMotor.setPower(speed);
                    intakeServoRoller.setPower(servoSpeed);
                    return false;
                }
            };}
    }
}
