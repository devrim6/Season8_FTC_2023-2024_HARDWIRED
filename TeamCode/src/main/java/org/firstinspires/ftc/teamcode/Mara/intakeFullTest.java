package org.firstinspires.ftc.teamcode.Mara;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Variables.DefVal;

import java.util.ArrayList;
import java.util.List;
@TeleOp(name ="intakeFullTest",group = "testing")
public class intakeFullTest extends LinearOpMode {
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();
    Servo intakeServoLeft;
    Servo intakeServoRight;
    DcMotor intakeMotor;
    CRServo intakeCRServo;
    int intakeMotorContor=0;
    int intakeCRServoContor=0;
    int levelContor=0;
    @Override
    public void runOpMode() throws InterruptedException {
        intakeMotor=hardwareMap.get(DcMotor.class,"intakeMotor");
        intakeServoLeft=hardwareMap.get(Servo.class,"intakeServoLeft");
        intakeServoRight=hardwareMap.get(Servo.class,"intakeServoRight");
        intakeCRServo=hardwareMap.get(CRServo.class,"intakeCRServo");
        waitForStart();
        while(opModeIsActive()){
            TelemetryPacket packet = new TelemetryPacket();
            if(gamepad1.a){
                intakeMotorContor++;
                if(intakeMotorContor==1){
                    intakeMotor.setPower(0.5);
                } else if(intakeMotorContor==2){
                    intakeMotor.setPower(0.5);
                    intakeMotorContor=0;
                }
            }
            if(gamepad1.b){
                intakeCRServoContor++;
                if(intakeCRServoContor==1){
                    intakeCRServo.setPower(0.5);
                } else if(intakeCRServoContor==2){
                    intakeCRServo.setPower(-0.5);
                    intakeCRServoContor=0;
                }
            }
            if(gamepad1.y){
                levelContor++;
                if(levelContor==1){
                    runningActions.add(maturiceLevel("Level1"));
                } else if(levelContor==2){
                    runningActions.add(maturiceLevel("Level2"));
                } else if(levelContor==3){
                    runningActions.add(maturiceLevel("Level3"));
                } else if(levelContor==4){
                    runningActions.add(maturiceLevel("Level4"));
                } else if(levelContor==5){
                    runningActions.add(maturiceLevel("Level5"));
                } else if(levelContor==6){
                    runningActions.add(maturiceLevel("Level6"));
                    levelContor=0;
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

            telemetry.addData("Level ",levelContor);
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
