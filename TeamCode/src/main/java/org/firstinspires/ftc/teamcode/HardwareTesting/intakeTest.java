package org.firstinspires.ftc.teamcode.HardwareTesting;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Variables.DefVal;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="intakeTest")
public class intakeTest extends LinearOpMode {
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();
    private Servo intakeServoLeft, intakeServoRight;
    private CRServo intakeServoRoller;
    private DcMotorEx intakeMotor;

    boolean isIntakeOn=false;
    int intakeLevel = 0;

    public void runOpMode() throws InterruptedException{

        intakeServoLeft = hardwareMap.get(Servo.class, "intakeServoLeft");
        intakeServoRight = hardwareMap.get(Servo.class, "intakeServoRight");
        intakeServoRoller = hardwareMap.get(CRServo.class, "intakeServoRoller");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        GamepadEx gamepadEx = new GamepadEx(gamepad2);

        intakeServoRight.setDirection(Servo.Direction.REVERSE);

        intakeServoRight.setPosition(DefVal.iLevel1);
        intakeServoLeft.setPosition(DefVal.iLevel1);

        telemetry.setMsTransmissionInterval(50);
        waitForStart();
        while(opModeIsActive() && !isStopRequested()){
            TelemetryPacket packet = new TelemetryPacket();

            //Intake level adjustment
            if(gamepadEx.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)){
                intakeLevel++; if(intakeLevel>6) intakeLevel=1;
                else runningActions.add(angle(intakeLevel));
            }
            if(gamepadEx.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)){
                intakeLevel--; if(intakeLevel<1) intakeLevel=6;
                else runningActions.add(angle(intakeLevel));
            }

            if(gamepadEx.wasJustPressed(GamepadKeys.Button.A)){
                isIntakeOn=!isIntakeOn;
                if(isIntakeOn) runningActions.add(powerOn());
                else runningActions.add(reverse());
            }

            // Update running actions
            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;
            dash.sendTelemetryPacket(packet);

            gamepadEx.readButtons();
            telemetry.addData("intakeLevel: ", intakeLevel);
            telemetry.update();
        }
    }
    public Action powerOn(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                intakeMotor.setMotorEnable();
                intakeMotor.setPower(DefVal.intakeMotorPower);
                intakeServoRoller.setPower(DefVal.intakeRollerPower);
                return false;
            }
        };}
    public Action stopTest(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                intakeMotor.setPower(0);
                intakeServoRoller.setPower(0); //todo: see if you should put intake at lvl6
                intakeMotor.setMotorDisable();
                return false;
            }
        };}
    public SequentialAction reverse(){
        return new SequentialAction(reverseBase(), new SleepAction(1.5), stopTest());
    }
    private Action reverseBase(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                intakeMotor.setPower(-DefVal.intakeMotorPower);
                intakeServoRoller.setPower(-DefVal.intakeRollerPower);
                return false; //Run for 1.5s then stop
            }
        };}

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
}
