package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args){
        MeepMeep meepMeep = new MeepMeep(800);
        RoadRunnerBotEntity AutoStangaRed = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(45, 50, Math.toRadians(180), Math.toRadians(180), 15.1)
                .followTrajectorySequence(driveShim ->
                        driveShim.trajectorySequenceBuilder(new Pose2d(-34.5, -58 , Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(-32.5, -33, Math.toRadians(80)))
                                .setReversed(true)
                                .splineTo(new Vector2d(-57, -36), Math.toRadians(180))
                                .waitSeconds(0) //wait half  a sec!! (maybe?)
                                //stackToMiddle
                                .setReversed(false)
                                .splineToLinearHeading(new Pose2d(-34, -58.5, Math.toRadians(0)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(-10, -58.5, Math.toRadians(0)), Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(25, -58.5, Math.toRadians(0)), Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(49, -36, Math.toRadians(0)), Math.toRadians(0))
                                //middleToStack
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(25, -58.5, Math.toRadians(0)), Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(-10, -58.5, Math.toRadians(0)), Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(-34, -58.5, Math.toRadians(0)), Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(-57, -36, Math.toRadians(0)), Math.toRadians(180))
                                //stackToLeft
                                .setReversed(false)
                                .splineToLinearHeading(new Pose2d(-34, -58.5, Math.toRadians(0)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(-10, -58.5, Math.toRadians(0)), Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(25, -58.5, Math.toRadians(0)), Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(49, -42, Math.toRadians(0)), Math.toRadians(0))
                                //leftToStack
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(25, -58.5, Math.toRadians(0)), Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(-10, -58.5, Math.toRadians(0)), Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(-34, -58.5, Math.toRadians(0)), Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(-57, -36, Math.toRadians(0)), Math.toRadians(180))
                                //stackToLeft
                                .setReversed(false)
                                .splineToLinearHeading(new Pose2d(-34, -58.5, Math.toRadians(0)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(-10, -58.5, Math.toRadians(0)), Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(25, -58.5, Math.toRadians(0)), Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(49, -42, Math.toRadians(0)), Math.toRadians(0))
                                //parkBottom
                                .setReversed(false)
                                .setTangent(Math.toRadians(-90))
                                .splineToLinearHeading(new Pose2d(49, -55, Math.toRadians(0)), Math.toRadians(-90))
                                .splineToLinearHeading(new Pose2d(57.5, -60, Math.toRadians(0)), Math.toRadians(0))
                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(AutoStangaRed)
                .start();
    }
}