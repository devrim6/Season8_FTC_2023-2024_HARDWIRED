package org.firstinspires.ftc.teamcode.HardwareTesting;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Variables.DefVal;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Test-Endgame-HangPlane", group = "testing")
public class endgameTest extends LinearOpMode {
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();
    DcMotorEx hangMotor;

    double PI = Math.PI;
    double GEAR_RATIO = 3*5;
    double WHEEL_DIAMETER_CM = 3.565;
    double GEAR_MOTOR_REV_ULTRAPLANETARY = 28 * GEAR_RATIO;
    final double TICKS_PER_CM_Z = GEAR_MOTOR_REV_ULTRAPLANETARY / (WHEEL_DIAMETER_CM * PI);
    int hangingCounter=0;

    public void runOpMode() throws InterruptedException{
        Servo planeLauncherServo = hardwareMap.get(Servo.class, "planeLauncherServo");
        hangMotor = hardwareMap.get(DcMotorEx.class, "hangMotor");
        GamepadEx gamepadEx = new GamepadEx(gamepad2);

        hangMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        hangMotor.setMotorDisable();

        planeLauncherServo.setPosition(0);

        boolean a=false;
        telemetry.setMsTransmissionInterval(50);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()){
            TelemetryPacket packet = new TelemetryPacket();

            if(gamepadEx.wasJustPressed(GamepadKeys.Button.B)){
                hangingCounter++; if(hangingCounter>3) hangingCounter=0;
                if(hangingCounter==1) runningActions.add(hangingEngage("up"));
                else if(hangingCounter==2) runningActions.add(hangingEngage("hang"));
                else if(hangingCounter==3){
                    runningActions.add(hangingEngage("normal"));
                    hangingCounter=0;
                }
            }
            if(gamepadEx.wasJustPressed(GamepadKeys.Button.A)){
                a=!a;
                if(a) planeLauncherServo.setPosition(DefVal.planeOn);
                else planeLauncherServo.setPosition(DefVal.planeOff);
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
            telemetry.addData("plane: ", a);
            telemetry.addData("hanging level: ", hangingCounter);
            telemetry.update();
        }
    }

    public Action hangingEngage(String state){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                hangMotor.setMotorEnable();
                switch(state){
                    case "up":
                        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        hangMotor.setTargetPosition((int)(DefVal.hangup*TICKS_PER_CM_Z));
                        hangMotor.setPower(1);
                        break;
                    case "hang":
                        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        hangMotor.setTargetPosition((int)(DefVal.hangHang*TICKS_PER_CM_Z));
                        hangMotor.setPower(0.3);
                        break;
                    case "normal":
                        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        hangMotor.setTargetPosition(0);
                        hangMotor.setPower(0.6);
                        break;
                }
                return false;
            }
        };
    }

}
