package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepStorage1 {
    MeepMeep meepMeep = new MeepMeep(700);
    RoadRunnerBotEntity Autonomie1 = new DefaultBotBuilder(meepMeep)   //Gata
            // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(45, 50, Math.toRadians(180), Math.toRadians(180), 15.1)
            .followTrajectorySequence(driveShim ->
                    driveShim.trajectorySequenceBuilder(new Pose2d(-34.5, -58 , Math.toRadians(90)))

                            .setReversed(false)
                            .splineToLinearHeading(new Pose2d(-29, -33, Math.toRadians(60)), Math.toRadians(60))
                            .waitSeconds(0.2)
                            .setReversed(true)
                            .splineToLinearHeading(new Pose2d(-53,-36,Math.toRadians(0)),Math.toRadians(90))
                            .waitSeconds(0.7) //ia un pixel
                            //1
                            .setTangent(Math.toRadians(90))
                            .splineToLinearHeading(new Pose2d(-38.39, -12.02,Math.toRadians(0)), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(27, -12.02,Math.toRadians(0)), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(48, -41.31,Math.toRadians(0)), Math.toRadians(0))
                            .waitSeconds(0.7)
                            .setReversed(true)
                            .splineToLinearHeading(new Pose2d(27, -12.02,Math.toRadians(0)), Math.toRadians(180))
                            .splineToLinearHeading(new Pose2d(-56.45, -12.02,Math.toRadians(0)), Math.toRadians(180))
                            .setReversed(false)
                            .waitSeconds(0.7)
                            .splineToLinearHeading(new Pose2d(27, -12.02,Math.toRadians(0)), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(48, -29.65,Math.toRadians(0)), Math.toRadians(0))
                            .waitSeconds(0.7)
                            .setReversed(true)
                            .splineToLinearHeading(new Pose2d(27, -12.02,Math.toRadians(0)), Math.toRadians(180))
                            .splineToLinearHeading(new Pose2d(-56.45, -12.02,Math.toRadians(0)), Math.toRadians(180))
                            .setReversed(false)
                            .waitSeconds(0.7)
                            .splineToLinearHeading(new Pose2d(27, -12.02,Math.toRadians(0)), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(48, -29.65,Math.toRadians(0)), Math.toRadians(0))
                            .waitSeconds(0.7)
                            //de final
                            .setReversed(true)
                            .setTangent(Math.toRadians(90))
                            .splineToSplineHeading(new Pose2d(48,-15,Math.toRadians(0)),Math.toRadians(90))
                            .build()
            );

    RoadRunnerBotEntity Autonomie2 = new DefaultBotBuilder(meepMeep)
            .setConstraints(45, 50, Math.toRadians(180), Math.toRadians(180), 15.1)
            .followTrajectorySequence(driveShim ->
                    driveShim.trajectorySequenceBuilder(new Pose2d(-34.5, -58 , Math.toRadians(90)))
                            .setReversed(false)
                            .splineToLinearHeading(new Pose2d(-29, -33, Math.toRadians(60)), Math.toRadians(60))
                            .waitSeconds(0.2)
                            .setReversed(true)
                            .splineToLinearHeading(new Pose2d(-53,-36,Math.toRadians(0)),Math.toRadians(180))
                            .waitSeconds(0.7) //ia un pixel
                            //1
                            .setTangent(Math.toRadians(90))
                            .setReversed(false)
                            .splineToLinearHeading(new Pose2d(-33,-59,Math.toRadians(0)),Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(27,-59,Math.toRadians(0)),Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(48,-42,Math.toRadians(0)),Math.toRadians(0))
                            .waitSeconds(0.7)
                            .setReversed(true)
                            .splineToLinearHeading(new Pose2d(27,-59,Math.toRadians(0)),Math.toRadians(180))
                            .splineToLinearHeading(new Pose2d(-33,-59,Math.toRadians(0)),Math.toRadians(180))
                            .splineToLinearHeading(new Pose2d(-53,-36,Math.toRadians(0)),Math.toRadians(180))
                            .waitSeconds(0.7)
                            .setReversed(false)
                            .splineToLinearHeading(new Pose2d(-33,-59,Math.toRadians(0)),Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(27,-59,Math.toRadians(0)),Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(48,-42,Math.toRadians(0)),Math.toRadians(0))
                            .waitSeconds(0.7)
                            .setReversed(true)
                            .splineToLinearHeading(new Pose2d(27,-59,Math.toRadians(0)),Math.toRadians(180))
                            .splineToLinearHeading(new Pose2d(-33,-59,Math.toRadians(0)),Math.toRadians(180))
                            .splineToLinearHeading(new Pose2d(-53,-36,Math.toRadians(0)),Math.toRadians(180))
                            .waitSeconds(0.7)
                            .setReversed(false)
                            .splineToLinearHeading(new Pose2d(-33,-59,Math.toRadians(0)),Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(27,-59,Math.toRadians(0)),Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(48,-42,Math.toRadians(0)),Math.toRadians(0))
                            .waitSeconds(0.7)

                            //de final
                            .setReversed(true)
                            .setTangent(Math.toRadians(-90))
                            .splineToSplineHeading(new Pose2d(48,-56,Math.toRadians(0)),Math.toRadians(270))
                            .build()
            );

    RoadRunnerBotEntity AutoTesting=new DefaultBotBuilder(meepMeep)
            .setConstraints(45, 50, Math.toRadians(180), Math.toRadians(180), 15.1)
            .followTrajectorySequence(driveShim ->
                    driveShim.trajectorySequenceBuilder(new Pose2d(-34.5, -58 , Math.toRadians(90)))
                            /*.splineToLinearHeading(new Pose2d(-34, 38,Math.toRadians(330)), Math.toRadians(90))
                            .setReversed(true)
                            .splineToLinearHeading(new Pose2d(-54, 36,Math.toRadians(0)), Math.toRadians(90))*/
                            /*.setReversed(false)
                            .splineToLinearHeading(new Pose2d(-33.16, -38, Math.toRadians(45)), Math.toRadians(90))
                            .lineToSplineHeading(new Pose2d(-56,-38,Math.toRadians(0)))*/
                            //.lineTo(new Vector2d(49,-15))
                            .setReversed(false)
                            .splineToLinearHeading(new Pose2d(-29, -33, Math.toRadians(60)), Math.toRadians(60))
                            .setReversed(true)
                            .splineToLinearHeading(new Pose2d(-53,-36,Math.toRadians(0)),Math.toRadians(180))
                            .setReversed(true)
                            .build()
            );
}