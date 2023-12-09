package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Variables.DefVal;

public class TestingOp extends LinearOpMode {
    /* Init whatever you need */
    TestingHwMap robot =  new TestingHwMap(hardwareMap);
    enum mode {
        TELEOP,
        HEADING_LOCK
    }
    TeleOpDrive.mode currentMode = TeleOpDrive.mode.TELEOP;
    int maxMenuVal = 12, menuVal=1;
    String component = "servo";

    public void runOpMode() throws InterruptedException{
        robot.gamepadInit(gamepad1,gamepad2);

        waitForStart();
        while(opModeIsActive() && !isStopRequested()) {
            String prevComp = component;
            component = DefVal.component;

            if(!component.equals(prevComp)) setTelemetry();

            if(robot.gamepad2Ex.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
                menuVal++;
                if(menuVal>maxMenuVal) menuVal=maxMenuVal;
                setTelemetry();
            } else if(robot.gamepad2Ex.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
                menuVal--;
                if(menuVal<1) menuVal=1;
                setTelemetry();
            }

            if(robot.gamepad2Ex.wasJustPressed(GamepadKeys.Button.A)) setComponent();

            switch (component){
                case "servo":
                    maxMenuVal=12;
                    break;
                case "motor":
                    maxMenuVal=8;
                    break;
                case "colorSensor":
                    maxMenuVal=2;
                    break;
            }
        }


    }

    public void setTelemetry(){
        telemetry.clearAll();
        switch (component){
            case "servo":
                telemetry.addLine("----Servo----");
                break;
            case "motor":
                telemetry.addLine("----Motor----");
                break;
            case "colorSensor":
                telemetry.addLine("----ColorSensor----");
                break;
        }

        for(int i=1;i<=maxMenuVal;++i){
            if(i==menuVal){
                telemetry.addData("Component selected: ", i);
            } else telemetry.addData("Component: ", i);
        }
    }
    public void setComponent(){
        telemetry.clearAll();
        switch (component){
            case "servo":
                Servo servo = hardwareMap.get(Servo.class, "servo"+menuVal);
                telemetry.addLine("----Servo " + menuVal + "----");
                break;
            case "motor":
                DcMotor motor = hardwareMap.get(DcMotor.class, "motor"+menuVal);
                telemetry.addLine("----Motor " + menuVal + "----");
                break;
            case "colorSensor":
                ColorSensor sensor = hardwareMap.get(ColorSensor.class, "colorSensor"+menuVal);
                telemetry.addLine("----ColorSensor " + menuVal + "----");
                break;
        }
    }
    boolean sensorWatch=true;
    public Action startSensorWatch(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                return false;
            }
        };
    }
}
