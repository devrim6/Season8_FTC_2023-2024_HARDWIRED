package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
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
import com.qualcomm.robotcore.hardware.Servo;
import com.arcrobotics.ftclib.hardware.SensorColor;


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

        /**
         * Takes the required height of the lift from the call and sets the slide to that position.
         * Ticks are calculated using TICKS_PER_CM_Z, which converts cm to motor ticks.
         * @param direction
         * @return false
         */
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
        public Intake() {}      // The constructor

        public Action powerOn(){
            return new Action() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    intakeMotor.setPower(0.5);
                    intakeServoRoller.setPower(-0.3);
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
                        case 6:             // is for auto init, goes up to 90 degrees perpendicular
                            intakeServoLeft.setPosition(1);
                            intakeServoRight.setPosition(-1);
                            break;
                    }                       // setPosition is async, action can be stopped immediately since
                    return false;           // it will run in another thread
                }
            };}

        boolean isIntakePowered=false;
        double currentTime = System.currentTimeMillis();
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
                    if(!upperSensorState.equals(HardwareMapping.ledState.OFF) && !bottomSensorState.equals(HardwareMapping.ledState.OFF)) {
                        if(System.currentTimeMillis()> currentTime + 500){ //Timer so that the bot is sure there are two pixels inside and doesn't have false positives
                            Actions.runBlocking(new ParallelAction(
                                    outtake.bottomHook("closed"),
                                    outtake.upperHook("closed"),
                                    reverse()
                            ));                                                     // Reverse intake to filter out
                            isIntakePowered = true;                                 // potential third pixel
                            return false;                                           // todo: implement beam break
                        }
                    } else currentTime = System.currentTimeMillis();
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

        public boolean isSensingOnline() {return !isIntakePowered;}
    }
}
