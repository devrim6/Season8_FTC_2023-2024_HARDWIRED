package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.arcrobotics.ftclib.hardware.SensorColor;

public class HardwareMapping {

    double PI = 3.1415;
    double GEAR_MOTOR_GOBILDA_312_TICKS = 537.7;
    double WHEEL_DIAMETER_CM = 3.565;
    double TICKS_PER_CM_Z = GEAR_MOTOR_GOBILDA_312_TICKS / (WHEEL_DIAMETER_CM * PI);

    enum liftHeight {
        GROUND,
        LOW,
        MIDDLE,
        HIGH
    }

    enum ledState {
        PURPLE,   //red
        GREEN,    //green
        YELLOW,   //amber
        WHITE,    //blinking between red and green
        OFF       //none
    }

    public Servo intakeServoLeft, intakeServoRight;
    public CRServo intakeServoRoller;
    public Servo planeLauncherServo;

    public Servo outtakePitchLeft, outtakePitchRight, outtakeYaw, outtakeRollLeft, outtakeRollRight,
            outtakeLatch, outtakeClawUpper, outtakeClawBottom;

    public DcMotorEx intakeMotor;
    public DcMotorEx hangMotor;
    public DcMotorEx slideMotorLeft;
    public DcMotorEx slideMotorRight;

    public GamepadEx gamepad1Ex, gamepad2Ex;

    public SensorColor bottomHookSensor, upperHookSensor;

    private DigitalChannel bottomLEDgreen, bottomLEDred, upperLEDgreen, upperLEDred;
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
        planeLauncherServo = hwMap.get(Servo.class, "planeLauncherServo");

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

        /* LEDs */ //Fiecare digital channel vine in perechi, n, n+1
        bottomLEDgreen = hwMap.get(DigitalChannel.class, "bottomLEDgreen");
        bottomLEDred = hwMap.get(DigitalChannel.class, "bottomLEDred");
        upperLEDgreen = hwMap.get(DigitalChannel.class, "upperLEDgreen");
        upperLEDred = hwMap.get(DigitalChannel.class, "upperLEDred");


        planeLauncherServo.setPosition(0.5);


    }

    public void gamepadInit(Gamepad gmpd1, Gamepad gmpd2){
        gamepad1Ex = new GamepadEx(gmpd1);
        gamepad2Ex = new GamepadEx(gmpd2);
    }

    public void resetEncoder(){
        hangMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public ledState checkColorRange(String sensor){
        float[] hsv = new float[3];
        switch (sensor){
            case "upper":
                hsv = upperHookSensor.RGBtoHSV(upperHookSensor.red(), upperHookSensor.green(), upperHookSensor.blue(), hsv);
                break;
            case "bottom":
                hsv = bottomHookSensor.RGBtoHSV(bottomHookSensor.red(), bottomHookSensor.green(), bottomHookSensor.blue(), hsv);
                break;
        }
        if(hsv[0] <= 320 && hsv[0] >= 280) {
            Actions.runBlocking(setLedColour(sensor, ledState.PURPLE));
            return ledState.PURPLE;
        }
        else if(hsv[0] <= 130 && hsv[0] >= 110){
            Actions.runBlocking(setLedColour(sensor, ledState.GREEN));
            return ledState.GREEN;
        }
        else if(hsv[0] <= 74 && hsv[0] >= 51){
            Actions.runBlocking(setLedColour(sensor, ledState.YELLOW));
            return ledState.YELLOW;
        }
        else if(hsv[1] <= 10 && hsv[1] >= 0){
            Actions.runBlocking(setLedColour(sensor, ledState.WHITE));
            return ledState.WHITE;
        }
        Actions.runBlocking(setLedColour(sensor, ledState.OFF));
        return ledState.OFF;
    }
    public Action setLedColour(String led, ledState colour){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                DigitalChannel led1 = null, led2 = null;
                if (led.equals("upper")) {
                    led1 = upperLEDred;                     //Fiecare digital channel vine in perechi: n, n+1
                    led2 = upperLEDgreen;
                } else if (led.equals("bottom")) {
                    led1 = bottomLEDred;
                    led2 = bottomLEDgreen;
                }
                ledColourDriver(colour, led1, led2);
                return false;
            }
        };
    }
    private boolean stopBlinking=false;
    public void ledColourDriver(ledState colour, DigitalChannel led1, DigitalChannel led2){
        switch (colour) {
            case OFF:
                stopBlinking=false;                         //stop white blinking
                led1.setState(false);                       //led1 is red
                led2.setState(false);                       //led2 is green
                break;
            case PURPLE:
                stopBlinking=false;
                led1.setState(true);
                led2.setState(false);
                break;
            case YELLOW:
                stopBlinking=false;
                led1.setState(true);
                led1.setState(true);
                break;
            case GREEN:
                stopBlinking=false;
                led1.setState(false);
                led2.setState(true);
                break;
            case WHITE:
                stopBlinking=true;                                          //start white blinking
                Actions.runBlocking(whitePixelBlink(led1, led2));           //separate thread, needs to run continously
                break;
        }
    }
    boolean whichLEDwhite=false;
    public Action whitePixelBlink(DigitalChannel led1, DigitalChannel led2){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(whichLEDwhite){
                    led1.setState(true);
                    led2.setState(false);               //todo: implement a proper timer, rn its spammy and seizure inducing, red card 110%
                    whichLEDwhite=false;
                } else {
                    led1.setState(false);
                    led2.setState(true);
                    whichLEDwhite=true;
                }
                return stopBlinking;
            }
        };
    }

    public Action launchPlane(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                planeLauncherServo.setPosition(0.1);
                return false;
            }
        };
    }
    public Action hangingEngage(String state){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                switch(state){
                    case "up":
                        hangMotor.setTargetPosition((int)(15*TICKS_PER_CM_Z));
                        hangMotor.setPower(1);
                        break;
                    case "hang":
                        hangMotor.setTargetPosition((int)(10*TICKS_PER_CM_Z));
                        //todo: set velocity? seems like setpower() already does that in runtoposition mode
                        hangMotor.setPower(0.3);
                        break;
                }
                return false;
            }
        };
    }

    public class Outtake {
        public Outtake() {}                 // The constructor

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
        public Action runToPosition(liftHeight direction){
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    switch(direction){
                        case HIGH:
                            slideMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            slideMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            slideMotorLeft.setTargetPosition((int)(24*TICKS_PER_CM_Z));
                            slideMotorRight.setTargetPosition(-(int)(24*TICKS_PER_CM_Z));
                            slideMotorRight.setPower(1);
                            slideMotorLeft.setPower(1);
                        case MIDDLE:
                            slideMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            slideMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            slideMotorLeft.setTargetPosition((int)(10*TICKS_PER_CM_Z));
                            slideMotorRight.setTargetPosition(-(int)(10*TICKS_PER_CM_Z));
                            slideMotorRight.setPower(1);
                            slideMotorLeft.setPower(1);
                        case LOW:
                            slideMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            slideMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            slideMotorLeft.setTargetPosition((int)(5*TICKS_PER_CM_Z));
                            slideMotorRight.setTargetPosition(-(int)(5*TICKS_PER_CM_Z));
                            slideMotorRight.setPower(1);
                            slideMotorLeft.setPower(1);
                        case GROUND:
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
                final double time = System.currentTimeMillis();
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    intakeMotor.setPower(-0.5);
                    intakeServoRoller.setPower(0.3);
                    return time < System.currentTimeMillis()+1500; //Run for 1.5s then stop
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
                    return false;           // setPosition is async, action can be stopped immediately since
                                            // it will run in another thread
                }
            };}
    }
}
