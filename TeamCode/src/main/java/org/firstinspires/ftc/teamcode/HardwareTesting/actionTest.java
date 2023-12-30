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
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="actionTest")
public class actionTest extends LinearOpMode {
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    private Servo intakeServoLeft, intakeServoRight;
    private String state = "not";
    private String a;

    public void runOpMode() throws InterruptedException{

        intakeServoLeft = hardwareMap.get(Servo.class, "intakeServoLeft");
        intakeServoRight = hardwareMap.get(Servo.class, "intakeServoRight");

        GamepadEx gamepadEx = new GamepadEx(gamepad2);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()){
            TelemetryPacket packet = new TelemetryPacket();

            if(gamepadEx.wasJustPressed(GamepadKeys.Button.A)) runningActions.add(new SequentialAction(
                    smth(),
                    new SleepAction(2),
                    smth2()
            ));

            if(gamepadEx.isDown(GamepadKeys.Button.B)) a = "pressed";
            else a = "not pressed";

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
            telemetry.addLine(state);
            telemetry.addLine(a);
            telemetry.update();
        }
    }

    public Action smth(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                intakeServoLeft.setPosition(0.3);
                intakeServoRight.setPosition(0.3);
                state = "started";
                return false;
            }
        };
    }
    public Action smth2(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                intakeServoLeft.setPosition(0);
                intakeServoRight.setPosition(0);
                state = "stopped";
                return false;
            }
        };
    }
}
