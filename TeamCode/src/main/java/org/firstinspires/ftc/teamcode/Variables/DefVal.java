package org.firstinspires.ftc.teamcode.Variables;

import com.acmerobotics.dashboard.config.Config;

@Config
public class DefVal {
    //in ms
    public static int endgameTime = 50000;

    public static double planeOff = 0;
    public static double planeOn = 1;

    public static double latchOpen = 0;
    public static double latchClosed = 0.5;

    public static double upperHookOpen = 1;
    public static double upperHookClosed = 0.85;
    public static double bottomHookClosed = 0.75;
    public static double bottomHookOpen = 0.9;

    //CM converted to ticks
    public static double LiftHIGH = 50;
    public static double LiftMIDDLE = 35;
    public static double LiftLOW = 20;
    public static double LiftGROUND = 0;

    public static double hangHang = 10;
    public static double hangup = 15;

    public static double intakeMotorPower = 0.65;
    public static double intakeRollerPower = 0.5;

    public static double iLevel1 = 0.08;
    public static double iLevel2 = 0.08;
    public static double iLevel3 = 0.13;
    public static double iLevel4 = 0.145;
    public static double iLevel5 = 0.16;
    public static double iLevel6 = 0.4;

    public static double yaw0 = 0.75;
    public static double yaw90 = 1.3;

    public static double pivot0 = 2.55;
    public static double pivot60 = 7.1;

    public static double roll0 = 0.7;
    public static double roll60 = 4.2;

    public static double pitchPositive = 10;
    public static double pitchNegative = -15;
    public static double TILT_POWER = 1;

    public static String component = "servo";
}