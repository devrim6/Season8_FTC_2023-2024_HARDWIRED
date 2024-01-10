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

    public static double upperHookOpen = 0;
    public static double upperHookClosed = 0.16;
    public static double bottomHookClosed = 0.1;
    public static double bottomHookOpen = 0.3;

    //CM converted to ticks
    public static double LiftHIGH = 24;
    public static double LiftMIDDLE = 10;
    public static double LiftLOW = 5;
    public static double LiftGROUND = 0;

    public static double hangHang = 10;
    public static double hangup = 15;

    public static double intakeMotorPower = 0.5;
    public static double intakeRollerPower = 0.3;

    public static double iLevel1 = 0.04;
    public static double iLevel2 = 0.06;
    public static double iLevel3 = 0.1;
    public static double iLevel4 = 0.13;
    public static double iLevel5 = 0.145;
    public static double iLevel6 = 0.25;

    public static double yaw0 = 3.25;
    public static double yaw90 = 1.05;

    public static double pivot0 = 2.8;
    public static double pivot60 = 7.1;

    public static double roll0 = 0.5;
    public static double roll60 = 4.3;

    public static double pitchPositive = 10;
    public static double pitchNegative = -15;
    public static double TILT_POWER = 1;

    public static String component = "servo";
}