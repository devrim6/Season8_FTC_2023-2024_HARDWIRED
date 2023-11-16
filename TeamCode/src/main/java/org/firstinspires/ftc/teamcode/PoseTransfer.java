package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import org.firstinspires.ftc.teamcode.HardwareMapping.ledState;

public class PoseTransfer {
    public static Pose2d currentPose = new Pose2d(58, -34.5, Math.toRadians(270));
    public static double glisieraTicks1 = 0;
    public static double glisieraTicks2 = 0;
    public static ledState bottomLedState = HardwareMapping.ledState.OFF;
    public static ledState upperLedState = HardwareMapping.ledState.OFF;
    public PoseTransfer() {}
}
