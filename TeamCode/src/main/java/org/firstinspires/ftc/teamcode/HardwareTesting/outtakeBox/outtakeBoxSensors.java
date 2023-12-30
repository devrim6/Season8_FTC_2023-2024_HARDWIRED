package org.firstinspires.ftc.teamcode.HardwareTesting.outtakeBox;

import android.graphics.Color;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.HardwareMapping;

import java.util.ArrayList;
import java.util.List;

@TeleOp (name = "SENSOR COLOUR TEST")
public class outtakeBoxSensors extends LinearOpMode {
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();
    ColorSensor upperHookSensor, bottomHookSensor;
    public void runOpMode() throws InterruptedException{
        /* Sensors */
        upperHookSensor = hardwareMap.get(ColorSensor.class, "upperHookSensor");
        bottomHookSensor = hardwareMap.get(ColorSensor.class, "bottomHookSensor");
        telemetry.setMsTransmissionInterval(50);
        GamepadEx gamepadEx = new GamepadEx(gamepad2);
        bottomHookSensor.enableLed(false);
        upperHookSensor.enableLed(false);
        waitForStart();
        while (opModeIsActive() && !isStopRequested()){
            TelemetryPacket packet = new TelemetryPacket();

            float[] hsv = new float[3];
            float[] hsv1 = new float[3];
            Color.RGBToHSV(bottomHookSensor.red(), bottomHookSensor.green(), bottomHookSensor.blue(), hsv);
            Color.RGBToHSV(upperHookSensor.red(), upperHookSensor.green(), upperHookSensor.blue(), hsv1);


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
            telemetry.addLine("\nBottom Sensor: \n");
            telemetry.addLine("H: " + hsv[0] + " S: " + hsv[1] + " V: " + hsv[2]);
            telemetry.addData("Colour: ", checkColorRange("bottom").toString());
            telemetry.addLine("\nUpper Sensor: \n");
            telemetry.addLine("H: " + hsv1[0] + " S: " + hsv1[1] + " V: " + hsv1[2]);
            telemetry.addData("Colour: ", checkColorRange("upper").toString());
            telemetry.update();
        }
    }
    public HardwareMapping.ledState checkColorRange(@NonNull String sensor){
        float[] hsv = new float[3];
        switch (sensor){
            case "upper":
                upperHookSensor.enableLed(true);
                Color.RGBToHSV(upperHookSensor.red(), upperHookSensor.green(), upperHookSensor.blue(), hsv);
                break;
            case "bottom":
                bottomHookSensor.enableLed(true);
                Color.RGBToHSV(bottomHookSensor.red(), bottomHookSensor.green(), bottomHookSensor.blue(), hsv);
                break;
        }

        return isInBounds(hsv);
    }
    public HardwareMapping.ledState isInBounds(@NonNull float[] a){
        int[] whiteLower = {75,0,99};
        int[] whiteUpper = {179,62,255};

        int[] greenLower = {40,40,40};
        int[] greenUpper = {70,255,255};

        int[] yellowLower = {30,125,150};
        int[] yellowUpper = {30,255,150};

        int[] purpleLower = {140,50,70};
        int[] purpleUpper = {160,250,250};

        if((a[0]>=whiteLower[0] && a[0]<=whiteUpper[0])
                && (a[1]>=whiteLower[1] && a[1]<=whiteUpper[1])
                && (a[2]>=whiteLower[2] && a[2]<=whiteUpper[2])) return HardwareMapping.ledState.WHITE;
        else if((a[0]>=greenLower[0] && a[0]<=greenUpper[0])
                && (a[1]>=greenLower[1] && a[1]<=greenUpper[1])
                && (a[2]>=greenLower[2] && a[2]<=greenUpper[2])) return HardwareMapping.ledState.GREEN;
        else if((a[0]>=yellowLower[0] && a[0]<=yellowUpper[0])
                && (a[1]>=yellowLower[1] && a[1]<=yellowUpper[1])
                && (a[2]>=yellowLower[2] && a[2]<=yellowUpper[2])) return HardwareMapping.ledState.YELLOW;
        else if((a[0]>=purpleLower[0] && a[0]<=purpleUpper[0])
                && (a[1]>=purpleLower[1] && a[1]<=purpleUpper[1])
                && (a[2]>=purpleLower[2] && a[2]<=purpleUpper[2])) return HardwareMapping.ledState.PURPLE;

        return HardwareMapping.ledState.OFF;
    }

}
