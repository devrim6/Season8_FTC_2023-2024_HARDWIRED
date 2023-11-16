package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepStorage {
    MeepMeep meepMeep = new MeepMeep(800);
    RoadRunnerBotEntity AutoStangaRedBottom = new DefaultBotBuilder(meepMeep)
            // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(45, 50, Math.toRadians(180), Math.toRadians(180), 15.1)
            .followTrajectorySequence(driveShim ->
                    driveShim.trajectorySequenceBuilder(new Pose2d(-34.5, -58 , Math.toRadians(90)))

                            //RIGHT
//                            .lineToLinearHeading(new Pose2d(-30, -33, Math.toRadians(60)))
//                            .setReversed(true)
//                            .splineTo(new Vector2d(-57, -36), Math.toRadians(180))

                            //MIDDLE
                            .lineToLinearHeading(new Pose2d(-32.5, -33, Math.toRadians(80)))
                            .setReversed(true)
                            .splineTo(new Vector2d(-57, -36), Math.toRadians(180))

                            //LEFT
//                            .lineToLinearHeading(new Pose2d(-43, -33, Math.toRadians(110)))
//                            .setReversed(true)
//                            .splineTo(new Vector2d(-57, -36), Math.toRadians(180))


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
    RoadRunnerBotEntity AutoDreaptaRedBottom = new DefaultBotBuilder(meepMeep)
            // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(45, 50, Math.toRadians(180), Math.toRadians(180), 15.1)
            .followTrajectorySequence(driveShim ->
                    driveShim.trajectorySequenceBuilder(new Pose2d(-34.5 + 2*24 , -58 , Math.toRadians(90)))

                            //RIGHT
//                            .lineToLinearHeading(new Pose2d(-30 + 2*24, -33, Math.toRadians(60)))
//                            .setReversed(true)
//                            .splineToLinearHeading(new Pose2d(49, -36, Math.toRadians(0)), Math.toRadians(0))

                            //MIDDLE
                            .lineToLinearHeading(new Pose2d(-32.5 + 2*24, -33, Math.toRadians(80)))
                            .setReversed(true)
                            .splineToLinearHeading(new Pose2d(49, -36, Math.toRadians(0)), Math.toRadians(0))


                            //LEFT
//                            .lineToLinearHeading(new Pose2d(-43 + 2*24, -33, Math.toRadians(110)))
//                            .setReversed(true)
//                            .splineToLinearHeading(new Pose2d(49, -36, Math.toRadians(0)), Math.toRadians(0))


                            .waitSeconds(0) //wait half  a sec!! (maybe?)
                            //middleToStack
                            .setReversed(true)
                            .splineToLinearHeading(new Pose2d(25, -58.5, Math.toRadians(0)), Math.toRadians(180))
                            .splineToLinearHeading(new Pose2d(-10, -58.5, Math.toRadians(0)), Math.toRadians(180))
                            .splineToLinearHeading(new Pose2d(-34, -58.5, Math.toRadians(0)), Math.toRadians(180))
                            .splineToLinearHeading(new Pose2d(-57, -36, Math.toRadians(0)), Math.toRadians(180))
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
    RoadRunnerBotEntity AutoStangaRedUpper = new DefaultBotBuilder(meepMeep)
            // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(45, 50, Math.toRadians(180), Math.toRadians(180), 15.1)
            .followTrajectorySequence(driveShim ->
                    driveShim.trajectorySequenceBuilder(new Pose2d(-34.5, -58 , Math.toRadians(90)))

                            //RIGHT
//                            .lineToLinearHeading(new Pose2d(-30, -33, Math.toRadians(60)))
//                            .setReversed(true)
//                            .splineTo(new Vector2d(-40, -36), Math.toRadians(180))
//                            .splineTo(new Vector2d(-57, -25), Math.toRadians(180))

                            //MIDDLE
                            .lineToLinearHeading(new Pose2d(-32.5, -33, Math.toRadians(80)))
                            .setReversed(true)
                            .splineTo(new Vector2d(-40, -36), Math.toRadians(180))
                            .splineTo(new Vector2d(-57, -25), Math.toRadians(180))

                            //LEFT
//                            .lineToLinearHeading(new Pose2d(-43, -33, Math.toRadians(110)))
//                            .setReversed(true)
//                            .splineTo(new Vector2d(-50, -36), Math.toRadians(180))
//                            .splineTo(new Vector2d(-57, -25), Math.toRadians(180))



                            //stackToMiddle
                            .setReversed(false)
                            .splineToLinearHeading(new Pose2d(-34, -10.5, Math.toRadians(0)), Math.toRadians(0))
                            .splineToSplineHeading(new Pose2d(-10, -10.5, Math.toRadians(0)), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(25, -10.5, Math.toRadians(0)), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(49, -36, Math.toRadians(0)), Math.toRadians(0))
                            //middleToStack
                            .setReversed(true)
                            .splineToLinearHeading(new Pose2d(25, -10.5, Math.toRadians(0)), Math.toRadians(180))
                            .splineToLinearHeading(new Pose2d(-10, -10.5, Math.toRadians(0)), Math.toRadians(180))
                            .splineToLinearHeading(new Pose2d(-34, -10.5, Math.toRadians(0)), Math.toRadians(180))
                            .splineToLinearHeading(new Pose2d(-57, -10.5, Math.toRadians(0)), Math.toRadians(180))
                            //stackToLeft
                            .setReversed(false)
                            .splineToLinearHeading(new Pose2d(-34, -10.5, Math.toRadians(0)), Math.toRadians(0))
                            .splineToSplineHeading(new Pose2d(-10, -10.5, Math.toRadians(0)), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(25, -10.5, Math.toRadians(0)), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(49, -42, Math.toRadians(0)), Math.toRadians(0))
                            //leftToStack
                            .setReversed(true)
                            .splineToLinearHeading(new Pose2d(25, -10.5, Math.toRadians(0)), Math.toRadians(180))
                            .splineToLinearHeading(new Pose2d(-10, -10.5, Math.toRadians(0)), Math.toRadians(180))
                            .splineToLinearHeading(new Pose2d(-34, -10.5, Math.toRadians(0)), Math.toRadians(180))
                            .splineToLinearHeading(new Pose2d(-57, -10.5, Math.toRadians(0)), Math.toRadians(180))
                            //stackToLeft
                            .setReversed(false)
                            .splineToLinearHeading(new Pose2d(-34, -10.5, Math.toRadians(0)), Math.toRadians(0))
                            .splineToSplineHeading(new Pose2d(-10, -10.5, Math.toRadians(0)), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(25, -10.5, Math.toRadians(0)), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(49, -42, Math.toRadians(0)), Math.toRadians(0))
                            //parkBottom
                            .setReversed(false)
                            .setTangent(Math.toRadians(-90))
                            .splineToLinearHeading(new Pose2d(49, -55, Math.toRadians(0)), Math.toRadians(-90))
                            .splineToLinearHeading(new Pose2d(57.5, -60, Math.toRadians(0)), Math.toRadians(0))
                            .build()
            );
    RoadRunnerBotEntity AutoDreaptaRedUpper = new DefaultBotBuilder(meepMeep) //todo: not finished in the slightest
            // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(45, 50, Math.toRadians(180), Math.toRadians(180), 15.1)
            .followTrajectorySequence(driveShim ->
                            driveShim.trajectorySequenceBuilder(new Pose2d(-34.5 + 2*24 , -58 , Math.toRadians(90)))

                                    //RIGHT
//                            .lineToLinearHeading(new Pose2d(-30 + 2*24, -33, Math.toRadians(60)))
//                            .setReversed(true)
//                            .splineToLinearHeading(new Pose2d(49, -36, Math.toRadians(0)), Math.toRadians(0))

                                    //MIDDLE
                                    .lineToLinearHeading(new Pose2d(-32.5 + 2*24, -33, Math.toRadians(80)))
                                    .setReversed(true)
                                    .splineToLinearHeading(new Pose2d(49, -36, Math.toRadians(0)), Math.toRadians(0))


                                    //LEFT
//                            .lineToLinearHeading(new Pose2d(-43 + 2*24, -33, Math.toRadians(110)))
//                            .setReversed(true)
//                            .splineToLinearHeading(new Pose2d(49, -36, Math.toRadians(0)), Math.toRadians(0))


                                    .waitSeconds(0) //wait half  a sec!! (maybe?)
                                    //middleToStack
                                    .setReversed(true)
                                    .splineToLinearHeading(new Pose2d(25, -10.5, Math.toRadians(0)), Math.toRadians(180))
                                    .splineToLinearHeading(new Pose2d(-10, -10.5, Math.toRadians(0)), Math.toRadians(180))
                                    .splineToLinearHeading(new Pose2d(-34, -10.5, Math.toRadians(0)), Math.toRadians(180))
                                    .splineToLinearHeading(new Pose2d(-57, -36, Math.toRadians(0)), Math.toRadians(180))
                                    //stackToMiddle
                                    .setReversed(false)
                                    .splineToLinearHeading(new Pose2d(-34, -10.5, Math.toRadians(0)), Math.toRadians(0))
                                    .splineToSplineHeading(new Pose2d(-10, -10.5, Math.toRadians(0)), Math.toRadians(0))
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

}
