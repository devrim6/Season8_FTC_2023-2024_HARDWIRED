package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.arcrobotics.ftclib.hardware.SensorColor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Variables.DefVal;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


public class HardwareMapping {
    /**
     * TICKS_PER_CM_Z converts a specified amount of cm when multiplied with another value to motor ticks.
     */
    double PI = 3.1415;
    double GEAR_MOTOR_GOBILDA_312_TICKS = 537.7;
    double WHEEL_DIAMETER_CM = 3.565;
    double TICKS_PER_CM_Z = GEAR_MOTOR_GOBILDA_312_TICKS / (WHEEL_DIAMETER_CM * PI);

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

    public Servo outtakeLatch, outtakeClawUpper, outtakeClawBottom;
    public ServoEx outtakePitchLeft, outtakePitchRight, outtakeYaw, outtakeRollLeft, outtakeRollRight;

    public DcMotorEx intakeMotor;
    public DcMotorEx hangMotor;
    public DcMotorEx slideMotorLeft;
    public DcMotorEx slideMotorRight;

    public GamepadEx gamepad1Ex, gamepad2Ex;

    public SensorColor bottomHookSensor, upperHookSensor;

    private DigitalChannel bottomLEDgreen, bottomLEDred, upperLEDgreen, upperLEDred;
    HardwareMap hwMap = null;
    public HardwareMapping(){}

    /**
     * Takes ahwMap from call and then initializes each and every robot component while also
     * setting up some basic values that will be used in both Autonomous and TeleOp
     * @param ahwMap
     */
    public void init(HardwareMap ahwMap){
        hwMap = ahwMap;

        /* Motoare baza sunt in MecanumDrive */
        /* Servos */
        intakeServoLeft = hwMap.get(Servo.class, "intakeServoLeft");
        intakeServoRight = hwMap.get(Servo.class, "intakeServoRight");
        intakeServoRoller = hwMap.get(CRServo.class, "intakeServoRoller");
        outtakePitchLeft = new SimpleServo(hwMap, "outtakePitchLeft", Math.toRadians(0), Math.toRadians(350));
        outtakePitchRight = new SimpleServo(hwMap, "outtakePitchRight", Math.toRadians(0), Math.toRadians(350));
        outtakeYaw = new SimpleServo(hwMap, "outtakeYaw", Math.toRadians(0), Math.toRadians(90));
        outtakeRollLeft = new SimpleServo(hwMap, "outtakeRollLeft", Math.toRadians(0), Math.toRadians(350));
        outtakeRollRight = new SimpleServo(hwMap, "outtakeRollRight", Math.toRadians(0), Math.toRadians(350));
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

        //Plane armed position
        planeLauncherServo.setPosition(DefVal.planeOff);

    }

    public void gamepadInit(Gamepad gmpd1, Gamepad gmpd2){
        gamepad1Ex = new GamepadEx(gmpd1);
        gamepad2Ex = new GamepadEx(gmpd2);
    }

    public void resetEncoderAuto(){
        hangMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slideMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Takes sensor position, converts RGB to HSV then compares with predetermined values to see
     * which colour the pixel is, or if there is one at all, then passes the data along to the
     * call AND switches the LED lights
     * @param sensor
     * @return ledState
     */
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

    /**
     *  Takes the position of the led and the colour it needs to be set at then passes it along
     *  to ledColourDriver
     * @param led
     * @param colour
     * @return End the Action
     */
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

    /**
     * Takes the colour and led position then sets the colour accordingly. White calls upon the
     * whitePixelBlink Action asynchroniously to flash the leds on and off in order
     * @param colour
     * @param led1
     * @param led2
     */
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

    /**
     * Takes the led positions then flashes red and green in sequence to signify that a white pixel is
     * in that place.
     * @param led1
     * @param led2
     * @return false
     */
    public Action whitePixelBlink(DigitalChannel led1, DigitalChannel led2){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(whichLEDwhite){
                    led1.setState(true);
                    led2.setState(false);               //todo: implement a proper timer, rn its spammy and seizure inducing, red card 110%
                    whichLEDwhite=false;                //todo: he he he haw de ce este asa de cacat codul acesta - Raul 12B
                } else {
                    led1.setState(false);
                    led2.setState(true);
                    whichLEDwhite=true;
                }
                return stopBlinking;                    //if true then continue. if false then stop
            }
        };
    }

    public Action launchPlane(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                planeLauncherServo.setPosition(DefVal.planeOn);
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
                        hangMotor.setTargetPosition((int)(DefVal.hangup*TICKS_PER_CM_Z));
                        hangMotor.setPower(1);
                        break;
                    case "hang":
                        hangMotor.setTargetPosition((int)(DefVal.hangHang*TICKS_PER_CM_Z));
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
                    outtakePitchLeft.turnToAngle(-angle);
                    outtakePitchRight.turnToAngle(angle2);
                    return false;
                }
            };}

        /**
         * Takes the specified value and rotates the outtake relative to the previous angle using rotateBy().
         * Keeps the outtake box angle at 60 degrees, but the method needs to be called when the box is already
         * at 60 degrees in order for that to work
         * @param angle
         * @return false
         */
        public Action pivotFixedRoll(double angle){
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    outtakePitchLeft.rotateBy(-angle);              //Should only be called when outtake is at 60degrees already
                    outtakePitchRight.rotateBy(angle);              //todo: try to think of a better way
                    outtakeRollLeft.rotateBy(-angle);               //naive implementation
                    outtakeRollRight.rotateBy(angle);
                    return false;
                }
            };}
        public Action yaw(double angle){
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    outtakeYaw.turnToAngle(angle);
                    return false;
                }
            };}
        public Action roll(double angle, double angle2){
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    outtakeRollLeft.turnToAngle(-angle);
                    outtakeRollRight.turnToAngle(angle2);
                    return false;
                }
            };}
        public Action latch(String state){
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    switch(state){
                        case "open":
                            outtakeLatch.setPosition(DefVal.latchOpen);
                            break;
                        case "closed" :
                            outtakeLatch.setPosition(DefVal.latchClosed);
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
        public Action runToPosition(liftHeight direction){
            this.currentHeight = direction;
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    slideMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slideMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    switch(direction){
                        case HIGH:
                            slideMotorLeft.setTargetPosition((int)(DefVal.LiftHIGH*TICKS_PER_CM_Z));
                            slideMotorRight.setTargetPosition(-(int)(DefVal.LiftHIGH*TICKS_PER_CM_Z));
                        case MIDDLE:
                            slideMotorLeft.setTargetPosition((int)(DefVal.LiftMIDDLE*TICKS_PER_CM_Z));
                            slideMotorRight.setTargetPosition(-(int)(DefVal.LiftMIDDLE*TICKS_PER_CM_Z));
                        case LOW:
                            slideMotorLeft.setTargetPosition((int)(DefVal.LiftLOW*TICKS_PER_CM_Z));
                            slideMotorRight.setTargetPosition(-(int)(DefVal.LiftLOW*TICKS_PER_CM_Z));
                        case GROUND:
                            slideMotorLeft.setTargetPosition((int)(DefVal.LiftGROUND*TICKS_PER_CM_Z));
                            slideMotorRight.setTargetPosition(-(int)(DefVal.LiftGROUND*TICKS_PER_CM_Z));
                    }
                    slideMotorRight.setPower(1);
                    slideMotorLeft.setPower(1);
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
                    intakeMotor.setPower(DefVal.intakeMotorPower);
                    intakeServoRoller.setPower(-DefVal.intakeRollerPower);
                    return false;
                }
            };}

        /**
         * Reverses the intake for 1.5s to filter out a possible third pixel.
         */
        public Action reverse(){
            final double time = System.currentTimeMillis();
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    intakeMotor.setPower(-DefVal.intakeMotorPower);
                    intakeServoRoller.setPower(DefVal.LiftHIGH);
                    return time < System.currentTimeMillis()+1500; //Run for 1.5s then stop
                }
            };}
        public Action stop(){
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    intakeMotor.setPower(0);
                    intakeServoRoller.setPower(0); //todo: see if you should put intake at lvl6
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
                            intakeServoLeft.setPosition(DefVal.iLevel1_1);
                            intakeServoRight.setPosition(DefVal.iLevel1_2);
                            break;
                        case 2:
                            intakeServoLeft.setPosition(DefVal.iLevel2_1);
                            intakeServoRight.setPosition(-DefVal.iLevel2_2);
                            break;
                        case 3:
                            intakeServoLeft.setPosition(DefVal.iLevel3_1);
                            intakeServoRight.setPosition(-DefVal.iLevel3_2);
                            break;
                        case 4:
                            intakeServoLeft.setPosition(DefVal.iLevel4_1);
                            intakeServoRight.setPosition(-DefVal.iLevel4_2);
                            break;
                        case 5:
                            intakeServoLeft.setPosition(DefVal.iLevel5_1);
                            intakeServoRight.setPosition(-DefVal.iLevel5_2);
                            break;
                        case 6:             // is for auto init, goes up to 90 degrees perpendicular
                            intakeServoLeft.setPosition(DefVal.iLevel6_1);
                            intakeServoRight.setPosition(-DefVal.iLevel6_2);
                            break;
                    }                       // setPosition is async, action can be stopped immediately since
                    return false;           // it will run in another thread
                }
            };}

        boolean isIntakePowered=false;
        double currentTime1 = System.currentTimeMillis(), currentTime2 = System.currentTimeMillis();
        Outtake outtake = new Outtake();

        /**
         * Starts the pixel sensing system. If it detects two pixels inside the outtake box for x amount of seconds
         * it closes the hooks and reverses the intake for y seconds to filter out a potential third pixel.
         * This is an action because we want to save on loop times by asynchronously running it in a trajectory.
         * @return isSensingOnline status
         */
        public Action sensingOn(){
            isIntakePowered = false;
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    ledState upperSensorState, bottomSensorState;
                    upperSensorState = checkColorRange("upper");
                    bottomSensorState = checkColorRange("bottom");
                    boolean upper = upperSensorState.equals(ledState.OFF), bottom = bottomSensorState.equals(ledState.OFF);

                    if(!upper) {
                        if(System.currentTimeMillis()> currentTime1 + 500){ //Timer so that the bot is sure there are two pixels inside and doesn't have false positives
                            Actions.runBlocking(outtake.upperHook("closed"));
                            a=true;                                         // Reverse intake to filter out
                        }
                    } else currentTime1 = System.currentTimeMillis();

                    if(!bottom) {
                        if(System.currentTimeMillis()> currentTime2 + 500){
                            Actions.runBlocking(outtake.bottomHook("closed"));
                            a=true;
                        }
                    } else currentTime2 = System.currentTimeMillis();

                    if(!upper && !bottom){
                        Actions.runBlocking(new SequentialAction(
                                new ParallelAction(
                                        outtake.bottomHook("closed"),
                                        outtake.upperHook("closed")
                                ),
                                reverse(),
                                stop()
                        ));                                                     // Reverse intake to filter out
                        isIntakePowered = true;
                        a = true;                                               // potential third pixel
                        return false;                                           // todo: implement beam break
                    }

                    return !isIntakePowered;
                }
            };
        }

        /**
         * Turns off the pixel sensing system.
         */
        public Action sensingOff(){
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    isIntakePowered=true;                   // This is an action because we need to use it inside a trajectory
                    return false;
                }
            };
        }

        public void setCurrentHook(boolean aa){
            a=aa;
        }

        public boolean isSensingOnline() {return !isIntakePowered;}

    }
}
