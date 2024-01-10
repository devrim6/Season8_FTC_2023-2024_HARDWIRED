package org.firstinspires.ftc.teamcode.Mara;

import androidx.annotation.NonNull;

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

@TeleOp(name="intakeLevelTest",group = "testing")
public class intakeLevelTest extends LinearOpMode {
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();
    Servo intakeServoLeft;
    Servo intakeServoRight;
    int increment=0;

    @Override
    public void runOpMode() throws InterruptedException {

        intakeServoLeft = hardwareMap.get(Servo.class, "intakeServoLeft");
        intakeServoRight = hardwareMap.get(Servo.class, "intakeServoRight");
        intakeServoRight.setDirection(Servo.Direction.REVERSE);
        intakeServoRight.setPosition(DefVal.iLevel1);
        intakeServoLeft.setPosition(DefVal.iLevel1);
        GamepadEx gamepadEx = new GamepadEx(gamepad1);
        waitForStart();

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();
            if(gamepadEx.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)){
                increment++; if(increment>6) increment=1;
                else runningActions.add(level(increment));
            }
            if(gamepadEx.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)){
                increment--; if(increment<1) increment=6;
                else runningActions.add(level(increment));
            }
            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;
            dash.sendTelemetryPacket(packet);

            telemetry.addData("Level ",increment);
            telemetry.update();
        }
    }
    public Action level(int stare){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                switch (stare){
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
                    case 6:
                        intakeServoLeft.setPosition(DefVal.iLevel6);
                        intakeServoRight.setPosition(DefVal.iLevel6);
                        break;
                }
                return false;
            }
        };
    }
}
