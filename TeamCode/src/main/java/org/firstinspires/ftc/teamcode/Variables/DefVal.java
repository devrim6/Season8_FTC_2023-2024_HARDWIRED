package org.firstinspires.ftc.teamcode.Variables;

import com.acmerobotics.dashboard.config.Config;

@Config
public class DefVal {
    //in ms
    public static int endgameTime = 50000;

    public static double planeOff = 0.5;
    public static double planeOn = 0.1;

    public static double latchOpen = 0.5;
    public static double latchClosed = 0;

    public static double upperHookOpen = 0.5;
    public static double upperHookClosed = 0;
    public static double bottomHookClosed = 0;
    public static double bottomHookOpen = 0.5;

    //CM converted to ticks
    public static double LiftHIGH = 24;
    public static double LiftMIDDLE = 10;
    public static double LiftLOW = 5;
    public static double LiftGROUND = 0;

    public static double hangHang = 10;
    public static double hangup = 15;

    public static double intakeMotorPower = 0.5;
    public static double intakeRollerPower = 0.3;

    public static double iLevel1 = 0;
    public static double iLevel2 = 0.2;
    public static double iLevel3 = 0.4;
    public static double iLevel4 = 0.6;
    public static double iLevel5 = 0.8;
    public static double iLevel6 = 1;

    public static double yaw0 = 0;
    public static double yaw90 = 90;

    public static double pivot60 = 60;
    public static double pivot0 = 0;

    public static double roll60 = 60;
    public static double roll0 = 0;

    public static double pitchPositive = 10;
    public static double pitchNegative = -15;
    public static double TILT_POWER = 1;
}