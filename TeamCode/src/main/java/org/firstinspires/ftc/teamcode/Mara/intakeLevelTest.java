package org.firstinspires.ftc.teamcode.Mara;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
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

        waitForStart();

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();
            if(gamepad1.a){
                increment++;
                if(increment==1){
                    runningActions.add(maturiceLevel("Level1"));
                } else if(increment==2){
                    runningActions.add(maturiceLevel("Level2"));
                } else if(increment==3){
                    runningActions.add(maturiceLevel("Level3"));
                } else if(increment==4){
                    runningActions.add(maturiceLevel("Level4"));
                } else if(increment==5){
                    runningActions.add(maturiceLevel("Level5"));
                } else if(increment==6){
                    runningActions.add(maturiceLevel("Level6"));
                    increment=0;
                }
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
    public Action maturiceLevel(String stare){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                switch (stare){
                    case "Level1":
                        intakeServoLeft.setPosition(DefVal.iLevel1);
                        intakeServoRight.setPosition(DefVal.iLevel1);
                    case "Level2":
                        intakeServoLeft.setPosition(DefVal.iLevel2);
                        intakeServoRight.setPosition(DefVal.iLevel2);
                    case "Level3":
                        intakeServoLeft.setPosition(DefVal.iLevel3);
                        intakeServoRight.setPosition(DefVal.iLevel3);
                    case "Level4":
                        intakeServoLeft.setPosition(DefVal.iLevel4);
                        intakeServoRight.setPosition(DefVal.iLevel4);
                    case "Level5":
                        intakeServoLeft.setPosition(DefVal.iLevel5);
                        intakeServoRight.setPosition(DefVal.iLevel5);
                    case "Level6":
                        intakeServoLeft.setPosition(DefVal.iLevel6);
                        intakeServoRight.setPosition(DefVal.iLevel6);
                }
                return false;
            }
        };
    }
}
