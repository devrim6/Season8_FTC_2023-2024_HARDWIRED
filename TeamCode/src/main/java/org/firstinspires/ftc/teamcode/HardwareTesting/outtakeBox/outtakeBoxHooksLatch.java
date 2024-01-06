package org.firstinspires.ftc.teamcode.HardwareTesting.outtakeBox;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Variables.DefVal;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Test-Outtake-ClawLatch", group = "testing")
public class outtakeBoxHooksLatch extends LinearOpMode {
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    public Servo outtakeLatch, outtakeClawUpper, outtakeClawBottom;

    public void runOpMode() throws InterruptedException{
        GamepadEx gamepadEx = new GamepadEx(gamepad2);
        telemetry.setMsTransmissionInterval(50);

        outtakeLatch = hardwareMap.get(Servo.class, "outtakeLatch");
        outtakeClawBottom = hardwareMap.get(Servo.class, "outtakeClawBottom");
        outtakeClawUpper = hardwareMap.get(Servo.class, "outtakeClawUpper");

        boolean uHook=false,bHook=false,latch=false;

        outtakeClawUpper.setDirection(Servo.Direction.REVERSE);
        outtakeClawBottom.setDirection(Servo.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()){
            TelemetryPacket packet = new TelemetryPacket();

            if(gamepadEx.wasJustPressed(GamepadKeys.Button.X)){
                bHook=!bHook;
                if(bHook) outtakeClawBottom.setPosition(DefVal.bottomHookClosed);
                else outtakeClawBottom.setPosition(DefVal.bottomHookOpen);
            }
            if(gamepadEx.wasJustPressed(GamepadKeys.Button.B)){
                uHook=!uHook;
                if(uHook) outtakeClawUpper.setPosition(DefVal.upperHookClosed);
                else outtakeClawUpper.setPosition(DefVal.upperHookOpen);
            }

            if(gamepadEx.wasJustPressed(GamepadKeys.Button.Y)){
                latch=!latch;
                if(latch) outtakeLatch.setPosition(DefVal.latchClosed);
                else outtakeLatch.setPosition(DefVal.latchOpen);
            }

            if (gamepadEx.wasJustPressed(GamepadKeys.Button.START)){
                outtakeClawBottom.setPosition(0);
                outtakeClawUpper.setPosition(0);
                outtakeLatch.setPosition(0);
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

            telemetry.addData("uHook: ", outtakeClawUpper.getPosition());
            telemetry.addData("bHook: ", outtakeClawBottom.getPosition());
            telemetry.addData("latch: ", outtakeLatch.getPosition());
            telemetry.update();
        }
    }
}
