package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.arcrobotics.ftclib.hardware.SensorColor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.HardwareTesting.outtakeBox.SensorValues;
import org.firstinspires.ftc.teamcode.Variables.DefVal;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.opencv.core.Scalar;


public class HardwareMapping {
    public boolean isAtTarget(double db, double target, double offset){
        return db <= (target + offset) && db >= (target - offset);
    }
    /**
     * TICKS_PER_CM_Z converts a specified amount of cm when multiplied with another value to motor ticks.
     */
    double PI = Math.PI;
    double GEAR_RATIO = 3*5;
    double WHEEL_DIAMETER_CM = 3.565;
    double GEAR_MOTOR_REV_ULTRAPLANETARY = 28 * GEAR_RATIO;
    final double TICKS_PER_CM_Z = GEAR_MOTOR_REV_ULTRAPLANETARY / (WHEEL_DIAMETER_CM * PI);

    public enum liftHeight {
        GROUND,
        LOW,
        MIDDLE,
        HIGH
    }

    public enum ledState {
        PURPLE,   //red
        GREEN,    //green
        YELLOW,   //amber
        WHITE,    //blinking between red and green
        OFF       //none
    }

    public Servo intakeServoLeft, intakeServoRight;
    public CRServo intakeServoRoller;
    public Servo planeLauncherServo;

    public Servo  outtakeClawUpper, outtakeClawBottom;
    public ServoEx outtakePitchLeft, outtakePitchRight,  outtakeRollLeft, outtakeRollRight;

    public DcMotorEx intakeMotor;
   // public DcMotorEx hangMotor;
    public DcMotorEx slideMotorLeft;
    public DcMotorEx slideMotorRight;

    public GamepadEx gamepad1Ex, gamepad2Ex;

    //private ColorSensor bottomHookSensor, upperHookSensor;

    HardwareMap hwMap = null;
    public HardwareMapping(){}

    /**
     * Takes ahwMap from call and then initializes each and every robot component while also
     * setting up some basic values that will be used in both Autonomous and TeleOp
     * @param ahwMap
     */
    public void init(@NonNull HardwareMap ahwMap){
        hwMap = ahwMap;

        /* Motoare baza sunt in MecanumDrive */
        /* Servos */
        intakeServoLeft = hwMap.get(Servo.class, "intakeServoLeft");
        intakeServoRight = hwMap.get(Servo.class, "intakeServoRight");
        intakeServoRoller = hwMap.get(CRServo.class, "intakeServoRoller");
        outtakePitchLeft = new SimpleServo(hwMap, "outtakePitchLeft", Math.toRadians(0), Math.toRadians(350));
        outtakePitchRight = new SimpleServo(hwMap, "outtakePitchRight", Math.toRadians(0), Math.toRadians(350));
        //outtakeYaw = new SimpleServo(hwMap, "outtakeYaw", Math.toRadians(0), Math.toRadians(90));
        outtakeRollLeft = new SimpleServo(hwMap, "outtakeRollLeft", Math.toRadians(0), Math.toRadians(350));
        outtakeRollRight = new SimpleServo(hwMap, "outtakeRollRight", Math.toRadians(0), Math.toRadians(350));
        //outtakeLatch = hwMap.get(Servo.class, "outtakeLatch");
        outtakeClawBottom = hwMap.get(Servo.class, "outtakeClawBottom");
        outtakeClawUpper = hwMap.get(Servo.class, "outtakeClawUpper");
        planeLauncherServo = hwMap.get(Servo.class, "planeLauncherServo");

        outtakePitchLeft.setInverted(false);
        outtakePitchRight.setInverted(true);
        outtakeRollLeft.setInverted(false);
        outtakeRollRight.setInverted(true );
        intakeServoRight.setDirection(Servo.Direction.REVERSE);

        /* Motors */
        intakeMotor = hwMap.get(DcMotorEx.class, "intakeMotor");
//        hangMotor = hwMap.get(DcMotorEx.class, "hangMotor");
        slideMotorLeft = hwMap.get(DcMotorEx.class, "slideMotorLeft");
        slideMotorRight = hwMap.get(DcMotorEx.class, "slideMotorRight");

        slideMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        hangMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        hangMotor.setMotorDisable();

        /* Sensors */
//        upperHookSensor = ahwMap.get(ColorSensor.class, "upperHookSensor");
//        bottomHookSensor = ahwMap.get(ColorSensor.class, "bottomHookSensor");

        //Plane armed position
        planeLauncherServo.setPosition(DefVal.planeOff);

    }

    public void gamepadInit(Gamepad gmpd1, Gamepad gmpd2){
        gamepad1Ex = new GamepadEx(gmpd1);
        gamepad2Ex = new GamepadEx(gmpd2);
    }

    public void resetEncoderAuto(){
//        hangMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        hangMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slideMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

//    /**
//     * Takes sensor position, converts RGB to HSV then compares with predetermined values to see
//     * which colour the pixel is, or if there is one at all, then passes the data along to the
//     * call AND switches the LED lights
//     * @param sensor
//     * @return ledState
//     */
//    public ledState checkColorRange(@NonNull String sensor){
//        float[] hsv = new float[3];
//        switch (sensor){
//            case "upper":
//                upperHookSensor.enableLed(true);
//                Color.RGBToHSV(upperHookSensor.red(), upperHookSensor.green(), upperHookSensor.blue(), hsv);
//                break;
//            case "bottom":
//                bottomHookSensor.enableLed(true);
//                Color.RGBToHSV(bottomHookSensor.red(), bottomHookSensor.green(), bottomHookSensor.blue(), hsv);
//                break;
//        }
//
//        //        Actions.runBlocking(setLedColour(sensor, state));
//
//        return isInBounds(hsv);
//    }

//    /**
//     * Checks what colour (if any) the inputted hsv belongs to.
//     * @param a
//     */
//    public HardwareMapping.ledState isInBounds(@NonNull float[] a){
//        int[] whiteLower = {SensorValues.WHL,SensorValues.WSL,SensorValues.WVL};
//        int[] whiteUpper = {SensorValues.WHH, SensorValues.WSH, SensorValues.WVH};
//
//        int[] greenLower = {SensorValues.GHL, SensorValues.GSL, SensorValues.GVL};
//        int[] greenUpper = {SensorValues.GHH, SensorValues.GSH, SensorValues.GVH};
//
//        int[] yellowLower = {SensorValues.YHL, SensorValues.YSL, SensorValues.YVL};
//        int[] yellowUpper = {SensorValues.YHH, SensorValues.YSH, SensorValues.YVH};
//
//        int[] purpleLower = {SensorValues.PHL, SensorValues.PSL, SensorValues.PVL};
//        int[] purpleUpper = {SensorValues.PHH, SensorValues.PSH, SensorValues.PVH};
//
//        if((a[0]>=whiteLower[0] && a[0]<=whiteUpper[0])
//                && (a[1]>=whiteLower[1] && a[1]<=whiteUpper[1])
//                && (a[2]>=whiteLower[2] && a[2]<=whiteUpper[2])) return HardwareMapping.ledState.WHITE;
//        else if((a[0]>=greenLower[0] && a[0]<=greenUpper[0])
//                && (a[1]>=greenLower[1] && a[1]<=greenUpper[1])
//                && (a[2]>=greenLower[2] && a[2]<=greenUpper[2])) return HardwareMapping.ledState.GREEN;
//        else if((a[0]>=yellowLower[0] && a[0]<=yellowUpper[0])
//                && (a[1]>=yellowLower[1] && a[1]<=yellowUpper[1])
//                && (a[2]>=yellowLower[2] && a[2]<=yellowUpper[2])) return HardwareMapping.ledState.YELLOW;
//        else if((a[0]>=purpleLower[0] && a[0]<=purpleUpper[0])
//                && (a[1]>=purpleLower[1] && a[1]<=purpleUpper[1])
//                && (a[2]>=purpleLower[2] && a[2]<=purpleUpper[2])) return HardwareMapping.ledState.PURPLE;
//
//        return HardwareMapping.ledState.OFF;
//    }

//    /**
//     *  Takes the position of the led and the colour it needs to be set at then passes it along
//     *  to ledColourDriver
//     * @param led
//     * @param colour
//     */
//    public Action setLedColour(String led, ledState colour){
//        return new Action() {
//            @Override
//            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                DigitalChannel led1 = null, led2 = null;
//                if (led.equals("upper")) {
//                    led1 = upperLEDred;                     //Fiecare digital channel vine in perechi: n, n+1
//                    led2 = upperLEDgreen;
//                } else if (led.equals("bottom")) {
//                    led1 = bottomLEDred;
//                    led2 = bottomLEDgreen;
//                }
//                ledColourDriver(colour, led1, led2);
//                return false;
//            }
//        };
//    }
//    private boolean stopBlinking=false;

//    /**
//     * Takes the colour and led position then sets the colour accordingly. White calls upon the
//     * whitePixelBlink Action asynchroniously to flash the leds on and off in order
//     * @param colour
//     * @param led1
//     * @param led2
//     */
//    public void ledColourDriver(ledState colour, DigitalChannel led1, DigitalChannel led2){
//        switch (colour) {
//            case OFF:
//                stopBlinking=false;                         //stop white blinking
//                led1.setState(false);                       //led1 is red
//                led2.setState(false);                       //led2 is green
//                break;
//            case PURPLE:
//                stopBlinking=false;
//                led1.setState(true);
//                led2.setState(false);
//                break;
//            case YELLOW:
//                stopBlinking=false;
//                led1.setState(true);
//                led1.setState(true);
//                break;
//            case GREEN:
//                stopBlinking=false;
//                led1.setState(false);
//                led2.setState(true);
//                break;
//            case WHITE:
//                stopBlinking=true;                                          //start white blinking
//                Actions.runBlocking(whitePixelBlink(led1, led2));           //separate thread, needs to run continously
//                break;
//        }
//    }
//
//    /**
//     * Takes the led positions then flashes red and green in sequence to signify that a white pixel is
//     * in that place.
//     * @param led1
//     * @param led2
//     * @return false
//     */
//    public Action whitePixelBlink(DigitalChannel led1, DigitalChannel led2){
//        return new Action() {
//            @Override
//            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                try {
//                    led1.setState(true);
//                    led2.setState(false);
//                    Thread.sleep(800);
//                    led1.setState(false);
//                    led2.setState(true);
//                    Thread.sleep(800);
//                } catch (InterruptedException e) {
//                    throw new RuntimeException(e);
//                }
//                return stopBlinking;                    //if true then continue. if false then stop
//            }
//        };
//    }

    public Action launchPlane(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                planeLauncherServo.setPosition(DefVal.planeOn);
                return false;
            }
        };
    }
//    public Action hangingEngage(String state){
//        return new Action() {
//            @Override
//            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                hangMotor.setMotorEnable();
//                switch(state){
//                    case "up":
//                        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                        hangMotor.setTargetPosition((int)(DefVal.hangup*TICKS_PER_CM_Z));
//                        hangMotor.setPower(1);
//                        break;
//                    case "hang":
//                        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//                        hangMotor.setTargetPosition((int)(DefVal.hangHang*TICKS_PER_CM_Z));
//                        hangMotor.setPower(0.3);
//                        break;
//                    case "normal":
//                        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//                        hangMotor.setTargetPosition(0);
//                        hangMotor.setPower(0.6);
//                        break;
//                }
//                return false;
//            }
//        };
//    }

    public class Outtake {
        public Outtake() {}                 // The constructor

        /**
         * Uses the outtake arms, not the box
         */
        public Action pivot(double angle){
            return new Action() {
                boolean init = true;
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    if(init){
                        outtakePitchLeft.turnToAngle(angle);
                        outtakePitchRight.turnToAngle(angle);
                        init = false;
                    }
                    return false;
                }
            };}

        /**
         * Takes the specified value and rotates the outtake relative to the previous angle using rotateBy().
         * Keeps the outtake box angle at 60 degrees, but the method needs to be called when the box is already
         * at 60 degrees in order for that to work
         * @param angle
         */
        public Action pivotFixedRoll(double angle){
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    outtakePitchLeft.rotateBy(angle);              //Should only be called when outtake is at 60degrees already
                    outtakePitchRight.rotateBy(angle);             //todo: try to think of a better way
                    outtakeRollLeft.rotateBy(angle);               //naive implementation
                    outtakeRollRight.rotateBy(angle);
                    return false;
                }
            };}
        public boolean isOuttakeRotated=false;
        /**
         * Rotates outtake box 0 or 90 degrees
         */
        /*public Action yaw(double angle){
            return new Action() {
                boolean init = true;
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    if(init){
                        //outtakeYaw.turnToAngle(angle);
                        //if(angle==DefVal.yaw0) isOuttakeRotated=false;
                        //else if(angle==DefVal.yaw90) isOuttakeRotated=true;
                        init = false;
                    }
                    return false;
                }
            };}
        /**
         * Uses the outtake box, not arms
         */
        public Action roll(double angle){
            return new Action() {
                boolean init = true;
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    if(init){
                        outtakeRollLeft.turnToAngle(angle);
                        outtakeRollRight.turnToAngle(angle);
                        init=false;
                    }
                    return false;
                }
            };}
        public Action latch(String state){
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    switch(state){
                        case "open":
                            //outtakeLatch.setPosition(DefVal.latchOpen);
                            break;
                        case "closed" :
                            //outtakeLatch.setPosition(DefVal.latchClosed);
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
                                outtakeClawUpper.setPosition(DefVal.upperHookOpen);
                                break;
                            case "closed" :
                                outtakeClawUpper.setPosition(DefVal.upperHookClosed);
                                break;
                        }
                        return false;
                    }
                };}
            public Action bottomHook(String state){
                return new Action() {
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        switch(state){
                            case "open":
                                outtakeClawBottom.setPosition(DefVal.bottomHookOpen);
                                break;
                            case "closed" :
                                outtakeClawBottom.setPosition(DefVal.bottomHookClosed);
                                break;
                        }
                        return false;
                    }
                };}

        liftHeight currentHeight = liftHeight.LOW;
        /**
         * Takes the required height of the lift from the call and sets the slide to that position.
         * Ticks are calculated using TICKS_PER_CM_Z, which converts cm to motor ticks.
         * @param direction
         * @return false
         */
        public Action runToPosition(String direction){
            return new Action() {
                boolean init = true;
                int ticks = 0;
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    switch(direction){
                        case "high":
                            ticks = (int)(DefVal.LiftHIGH*TICKS_PER_CM_Z);
                            slideMotorLeft.setTargetPosition(ticks);
                            slideMotorRight.setTargetPosition(ticks);
                            slideMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            slideMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            slideMotorRight.setPower(1);
                            slideMotorLeft.setPower(1);
                            break;
                        case "middle":
                            ticks = (int)(DefVal.LiftMIDDLE*TICKS_PER_CM_Z);
                            slideMotorLeft.setTargetPosition(ticks);
                            slideMotorRight.setTargetPosition(ticks);
                            slideMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            slideMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            slideMotorRight.setPower(1);
                            slideMotorLeft.setPower(1);
                            break;
                        case "low":
                            ticks = (int)(DefVal.LiftLOW*TICKS_PER_CM_Z);
                            slideMotorLeft.setTargetPosition(ticks);
                            slideMotorRight.setTargetPosition(ticks);
                            slideMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            slideMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            slideMotorRight.setPower(1);
                            slideMotorLeft.setPower(1);
                            break;
                        case "ground":
                            ticks = (int)(DefVal.LiftGROUND*TICKS_PER_CM_Z);
                            slideMotorLeft.setTargetPosition(ticks);
                            slideMotorRight.setTargetPosition(ticks);
                            slideMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            slideMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            slideMotorRight.setPower(1);
                            slideMotorLeft.setPower(1);
                            break;
                    }
                    init = false;
                    return false;
                }
            };}
    }

    public static boolean a=false;

    public class Intake {
        public Intake() {}      // The constructor

        public Action powerOn(){
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    intakeMotor.setMotorEnable();
                    intakeMotor.setPower(-DefVal.intakeMotorPower);
                    intakeServoRoller.setPower(DefVal.intakeRollerPower);
                    return false;
                }
            };}

        /**
         * Reverses the intake for 1.5s to filter out a possible third pixel.
         */
        public SequentialAction reverse(){
            return new SequentialAction(reverseBase(), new SleepAction(1.5), stop());
        }

        private Action reverseBase(){
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    intakeMotor.setPower(DefVal.intakeMotorPower);
                    intakeServoRoller.setPower(-DefVal.intakeRollerPower);
                    return false; //Run for 1.5s then stop
                }
            };}
        public Action stop(){
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    intakeMotor.setPower(0);
                    intakeServoRoller.setPower(0); //todo: see if you should put intake at lvl6
                    intakeMotor.setMotorDisable();
                    return false;
                }
            };}

        /**
         * Takes the required level from the call and sets the intake to that specified angle. The angles
         * are relative to a 5 pixel stack height, level 5 being enough to only touch the last pixel on a
         * full stack.
         *
         * 6 is 90 degrees perpendicular or something to that extent, to make intake go up up up up up
         * @param level
         */
        public Action angle(int level){
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    switch (level){
                        case 1:
                            intakeServoLeft.setPosition(DefVal.iLevel1);
                            intakeServoRight.setPosition(DefVal.iLevel1);
                            break;
                        case 2:
                            intakeServoLeft.setPosition(DefVal.iLevel2);
                            intakeServoRight.setPosition(DefVal.iLevel2);
                            break;
                        case 3:
                            intakeServoLeft.setPosition(DefVal.iLevel3);
                            intakeServoRight.setPosition(DefVal.iLevel3);
                            break;
                        case 4:
                            intakeServoLeft.setPosition(DefVal.iLevel4);
                            intakeServoRight.setPosition(DefVal.iLevel4);
                            break;
                        case 5:
                            intakeServoLeft.setPosition(DefVal.iLevel5);
                            intakeServoRight.setPosition(DefVal.iLevel5);
                            break;
                        case 6:             // is for auto init, goes up to 90 degrees perpendicular
                            intakeServoLeft.setPosition(DefVal.iLevel6);
                            intakeServoRight.setPosition(DefVal.iLevel6);
                            break;
                    }                       // setPosition is async, action can be stopped immediately since
                    return false;           // it will run in another thread
                }
            };}

        private boolean isIntakePowered=false;
        boolean upperClosed=false,bottomClosed=false;
        double currentTime1 = System.currentTimeMillis(), currentTime2 = System.currentTimeMillis();
        Outtake outtake = new Outtake();

//        /**
//         * Starts the pixel sensing system. If it detects two pixels inside the outtake box for x amount of seconds
//         * it closes the hooks and reverses the intake for y seconds to filter out a potential third pixel.
//         * This is an action because we want to save on loop times by asynchronously running it in a trajectory.
//         * @return isSensingOnline status
//         */
//        public Action sensingOn(){
//            isIntakePowered = false;
//            bottomClosed=false; upperClosed=false;
//            //currentTime1=currentTime2=System.currentTimeMillis();
//            return new Action() {
//                @Override
//                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                    ledState upperSensorState, bottomSensorState;
//                    upperSensorState = checkColorRange("upper");
//                    bottomSensorState = checkColorRange("bottom");
//                    boolean upper = upperSensorState.equals(ledState.OFF), bottom = bottomSensorState.equals(ledState.OFF);
//
//                    if(!upper) {
//                        if(System.currentTimeMillis()> currentTime1 + 500){ // No false positives (maybe)
//                            Actions.runBlocking(outtake.upperHook("closed"));
//                            a = true;
//                            upperClosed=true;
//                        }
//                    } else{
//                        currentTime1 = System.currentTimeMillis();
//                        upperClosed = false;
//                    }
//
//                    if(!bottom) {
//                        if(System.currentTimeMillis()> currentTime2 + 500){
//                            Actions.runBlocking(outtake.bottomHook("closed"));
//                            a = true;
//                            bottomClosed = true;
//                        }
//                    } else{
//                        currentTime2 = System.currentTimeMillis();
//                        bottomClosed = false;
//                    }
//
//                    if(bottomClosed && upperClosed){
//                        Actions.runBlocking(new SequentialAction(
//                                new ParallelAction(
//                                        outtake.bottomHook("closed"),
//                                        outtake.upperHook("closed")
//                                ),
//                                reverse()
//                        ));                                                     // Reverse intake to filter out
//                        isIntakePowered = true;                                 // potential third pixel
//                        TeleOpDrive.isIntakePowered=false;
//                        a = true;
//                        return false;                                           // todo: implement beam break
//                    }
//
//                    return !isIntakePowered;
//                }
//            };
//        }
//        /**
//         * Turns off the pixel sensing system and sensor lights.
//         */
//        public Action sensingOff(){
//            return new Action() {
//                @Override
//                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
//                    isIntakePowered=true;                   // This is an action because we need to use it inside a trajectory
//                    upperHookSensor.enableLed(false);
//                    bottomHookSensor.enableLed(false);
//                    return false;
//                }
//            };
//        }

        public void setCurrentHook(boolean aa){
            a=aa;
        }

        public boolean isSensingOnline() {return !isIntakePowered;}

    }

    public class Auto{
        public boolean isTrajGoing;
        public Auto(){
            isTrajGoing=false;
        }

        /**
         * Tracks when a trajectory has ended (or any action really). Is used with SequentialAction.
         */
        public Action trajEnd(){
            return telemetryPacket -> {
                isTrajGoing=false;
                return false;
            };
        }
        public Action trajStart(){
            return telemetryPacket -> {
                isTrajGoing=true;
                return false;
            };
        }

        /**
         * Takes an action (in most cases a trajectory action) and keeps track of when it starts and ends. Can be used
         * for finite state systems.
         * @param traj
         */
        public SequentialAction followTrajectoryAndStop(Action traj){
            return new SequentialAction(trajStart(), traj, trajEnd());
        }

    }
}