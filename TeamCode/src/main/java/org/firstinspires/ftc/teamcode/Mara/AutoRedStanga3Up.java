package org.firstinspires.ftc.teamcode.Mara;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SafePathBuilder;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Mara.HardwareMapp;

@Autonomous(name = "AutoRedStanga3Up")
public class AutoRedStanga3Up extends LinearOpMode {

    HardwareMapp Robot = new HardwareMapp();
    private MecanumDrive drive;
    private Pose2d cPose;
    @Override
    public void runOpMode() throws InterruptedException {


        Robot.init(hardwareMap);
        //Robot.gamepadInit(gamepad1, gamepad2);

        Pose2d firstPose=new Pose2d(-34.5,-58,90);
        Pose2d RightLane=new Pose2d(-29,-33,60);
        Pose2d LeftStack=new Pose2d(-53,-36,0);
        Pose2d BackboardRight=new Pose2d(48,-41.31,0);
        Pose2d RightStack=new Pose2d(-56.45,-12.02,0);
        Pose2d BackboardLeft=new Pose2d(48,-29.65,0);

        cPose=firstPose;

        Action TrajRightLane = drive.actionBuilder(cPose)  //Traiectorie pana la linia din dreapta
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-29, -33, Math.toRadians(60)), Math.toRadians(60))
                .build();

        cPose=RightLane;

        Action TrajLeftStack = drive.actionBuilder(cPose)  //Traiectorie pana la Stackul din stanga
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-56.45,-36,Math.toRadians(0)),Math.toRadians(90))
                .afterDisp(1, new SequentialAction(  //1 inch = 2,54 cm
                        new ParallelAction(
                                Robot.hook1("open"),
                                Robot.Intake("in"),
                                Robot.maturiceOpen_Close("open"),
                                Robot.maturiceLevel("Level5")
                        )
                ))
                .build();

        cPose=LeftStack;

        Action TrajToBackboardRight = drive.actionBuilder(cPose)  //Traiectorie pana la backboard dreapta
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-38.39, -12.02, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(27, -12.02, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(48, -41.31, Math.toRadians(0)), Math.toRadians(0))
                /*.afterDisp(10, new SequentialAction(
                        Robot.misum("MIDDLE"),
                        Robot.turnOutake("turn"),
                        Robot.openOutake("open"),
                        Robot.openOutake("close"),
                        Robot.turnOutake("noTurn"),
                        Robot.misum("GROUND")
                ))*/
                .afterDisp(1,new SequentialAction(  //1 inch = 2,54 cm, (3 1/2), 1=23 inch
                        Robot.misumHeight("MIDDLE"),
                        new SleepAction(0.1),
                        Robot.turnOuttake("turn")
                ))
                .build();

        cPose=BackboardRight;

        //Level 1 2 3 4 5 6
        Action TrajRightStackLevel6 = drive.actionBuilder(cPose)  //Traiectorie pana la stackul din dreapta nivel 6 (teoretic nu ia pixel)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(27, -12.02, Math.toRadians(0)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-56.45, -12.02, Math.toRadians(0)), Math.toRadians(180))
                /*.afterDisp(10, new SequentialAction(
                        new ParallelAction(
                                Robot.Intake("in"),
                                Robot.maturiceOpen_Close("in"),
                                Robot.maturiceLevel("Level6")
                        ),
                        Robot.maturiceOpen_Close("off")
                ))*/
                .afterDisp(40,new SequentialAction(  //40 inch, ca sa treaca de truss
                        new ParallelAction(
                                Robot.Intake("in"),
                                Robot.maturiceOpen_Close("in"),
                                Robot.maturiceLevel("Level6")
                                //Robot.maturiceOpen_Close("off")
                        )
                ))
                .build();

        cPose=BackboardRight;

        Action TrajRightStackLevel5 = drive.actionBuilder(cPose)  //Traiectorie pana la stackul din dreapta nivel 5
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(27, -12.02, Math.toRadians(0)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-56.45, -12.02, Math.toRadians(0)), Math.toRadians(180))
                /*.afterDisp(10, new SequentialAction(
                        new ParallelAction(
                                Robot.Intake("in"),
                                Robot.maturiceOpen_Close("in"),
                                Robot.maturiceLevel("Level5")
                        ),
                        Robot.maturiceOpen_Close("off")
                ))*/
                .afterDisp(40,new SequentialAction(
                        new ParallelAction(
                                Robot.Intake("in"),
                                Robot.maturiceOpen_Close("in"),
                                Robot.maturiceLevel("Level5")
                                //Robot.maturiceOpen_Close("off")
                        )
                ))
                .build();

        cPose=BackboardRight;

        Action TrajRightStackLevel4 = drive.actionBuilder(cPose)  //Traiectorie pana la stackul din dreapta nivel 4
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(27, -12.02, Math.toRadians(0)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-56.45, -12.02, Math.toRadians(0)), Math.toRadians(180))
                /*.afterDisp(10, new SequentialAction(
                        new ParallelAction(
                                Robot.Intake("in"),
                                Robot.maturiceOpen_Close("in"),
                                Robot.maturiceLevel("Level4")
                        ),
                        Robot.maturiceOpen_Close("off")
                ))*/
                .afterDisp(40,new SequentialAction(
                        new ParallelAction(
                                Robot.Intake("in"),
                                Robot.maturiceOpen_Close("in"),
                                Robot.maturiceLevel("Level4")
                                //Robot.maturiceOpen_Close("off")
                        )
                ))
                .build();

        cPose=BackboardRight;

        Action TrajRightStackLevel3 = drive.actionBuilder(cPose)  //Traiectorie pana la stackul din dreapta nivel 3
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(27, -12.02, Math.toRadians(0)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-56.45, -12.02, Math.toRadians(0)), Math.toRadians(180))
                /*.afterDisp(10, new SequentialAction(
                        new ParallelAction(
                                Robot.Intake("in"),
                                Robot.maturiceOpen_Close("in"),
                                Robot.maturiceLevel("Level3")
                        ),
                        Robot.maturiceOpen_Close("off")
                ))*/
                .afterDisp(40,new SequentialAction(
                        new ParallelAction(
                                Robot.Intake("in"),
                                Robot.maturiceOpen_Close("in"),
                                Robot.maturiceLevel("Level3")
                                //Robot.maturiceOpen_Close("off")
                        )
                ))
                .build();

        cPose=BackboardRight;

        Action TrajRightStackLevel2 = drive.actionBuilder(cPose)  //Traiectorie pana la stackul din dreapta nivel 2
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(27, -12.02, Math.toRadians(0)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-56.45, -12.02, Math.toRadians(0)), Math.toRadians(180))
                /*.afterDisp(10, new SequentialAction(
                        new ParallelAction(
                                Robot.Intake("in"),
                                Robot.maturiceOpen_Close("in"),
                                Robot.maturiceLevel("Level2")
                        ),
                        Robot.maturiceOpen_Close("off")
                ))*/
                .afterDisp(40,new SequentialAction(
                        new ParallelAction(
                                Robot.Intake("in"),
                                Robot.maturiceOpen_Close("in"),
                                Robot.maturiceLevel("Level2")
                                //Robot.maturiceOpen_Close("off")
                        )
                ))
                .build();

        cPose=BackboardRight;

        Action TrajRightStackLevel1 = drive.actionBuilder(cPose)  //Traiectorie pana la stackul din dreapta nivel 1
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(27, -12.02, Math.toRadians(0)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-56.45, -12.02, Math.toRadians(0)), Math.toRadians(180))
                /*.afterDisp(10, new SequentialAction(
                        new ParallelAction(
                                Robot.Intake("in"),
                                Robot.maturiceOpen_Close("in"),
                                Robot.maturiceLevel("Level1")
                        ),
                        Robot.maturiceOpen_Close("off")
                ))*/
                .afterDisp(40,new SequentialAction(
                        new ParallelAction(
                                Robot.Intake("in"),
                                Robot.maturiceOpen_Close("in"),
                                Robot.maturiceLevel("Level1")
                                //Robot.maturiceOpen_Close("off")
                        )
                ))
                .build();

        cPose=RightStack;

        Action TrajToBackboardLeft = drive.actionBuilder(cPose)  //Traiectorie pana la backboard stanga
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(27, -12.02, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(48, -29.65, Math.toRadians(0)), Math.toRadians(0))
                /*.afterDisp(10, new ParallelAction(
                        new SequentialAction(
                                new ParallelAction(
                                        Robot.misum("MIDDLE")
                                ),
                                Robot.turnOutake("turn"),
                                Robot.openOutake("open")
                        ),
                        Robot.openOutake("close"),
                        Robot.turnOutake("noTurn"),
                        Robot.misum("GROUND")
                ))*/
                .afterDisp(1,new SequentialAction(
                        Robot.misumHeight("MIDDLE"),
                        new SleepAction(0.1),
                        Robot.turnOuttake("turn")
                            /*Robot.openOutake("open"),
                            Robot.openOutake("close"),
                            Robot.turnOutake("noTurn"),
                            Robot.misum("GROUND")*/
                ))
                .build();

        cPose=BackboardLeft;

        Action ParkingLeft = drive.actionBuilder(cPose)  //Traiectorie parcare stanga
                .setReversed(true)
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(48, -15, Math.toRadians(0)), Math.toRadians(90))
                .afterDisp(0.1, new SequentialAction(
                        new ParallelAction(
                                Robot.hook1("open"),
                                Robot.hook2("open")
                        ),
                        //new SleepAction(0.2),
                        Robot.openOuttake("close"),
                        new SleepAction(0.1),
                        Robot.turnOuttake("noTurn"),
                        new ParallelAction(
                                Robot.maturiceOpen_Close("off"),
                                Robot.Intake("off"),
                                Robot.maturiceLevel("Level1")
                        ),
                        Robot.misumHeight("GROUND")
                ))
                .build();

        waitForStart();
        Actions.runBlocking(new SequentialAction( //Face actiunile una dupa cealalta
                Robot.hook2("close"), //pixelul din cuva

                TrajRightLane,
                new SleepAction(0.1),

                TrajLeftStack,
                new SleepAction(0.7),
                Robot.hook1("close"),  //Pixelul de jos (in cel de sus este deja un pixel)
                new ParallelAction(
                        Robot.maturiceOpen_Close("off"),
                        Robot.Intake("off")   //Daca ia prea mult timp bag urmatoarea traiectorie in ParallelAction
                ),

                TrajToBackboardRight,
                Robot.openOuttake("open"),
                new SleepAction(0.1),
                new ParallelAction(
                        Robot.hook1("open"),
                        Robot.hook2("open")      //Las pixelii sa cada
                ),
                new SleepAction(0.1),
                    /*new ParallelAction(
                            Robot.hook1("close"),
                            Robot.hook2("close")
                    ),*/                                 //Las deschis pentru urmatorii pixeli
                new SleepAction(0.1),
                Robot.openOuttake("close"),
                new SleepAction(0.1),
                Robot.turnOuttake("noTurn"),
                new SleepAction(0.1),
                Robot.misumHeight("GROUND"),

                TrajRightStackLevel4,                //Level 4 ca sa imi ia primii 2 pixeli din stack ul din dreapta
                new SleepAction(0.7),
                new ParallelAction(
                        Robot.hook1("close"),
                        Robot.hook2("close")
                ),
                new ParallelAction(
                        Robot.maturiceOpen_Close("off"),
                        Robot.Intake("off")
                ),

                TrajToBackboardLeft,
                Robot.openOuttake("open"),
                new SleepAction(0.1),
                new ParallelAction(
                        Robot.hook1("open"),
                        Robot.hook2("open")      //Las pixelii sa cada
                ),
                new SleepAction(0.1),
                    /*new ParallelAction(
                            Robot.hook1("close"),
                            Robot.hook2("close")
                    ),*/                                  //Las deschis pentru urmatorii pixeli
                new SleepAction(0.1),
                Robot.openOuttake("close"),
                new SleepAction(0.1),
                Robot.turnOuttake("noTurn"),
                new SleepAction(0.1),
                Robot.misumHeight("GROUND"),

                TrajRightStackLevel2,                 //Level 2 ca sa imi ia al doilea si al treilea pixel din stack ul din dreapta
                new SleepAction(0.7),
                new ParallelAction(
                        Robot.hook1("close"),
                        Robot.hook2("close")
                ),
                new ParallelAction(
                        Robot.maturiceOpen_Close("off"),
                        Robot.Intake("off")
                ),

                TrajToBackboardLeft,
                Robot.openOuttake("open"),
                new SleepAction(0.1),
                new ParallelAction(
                        Robot.hook1("open"),
                        Robot.hook2("open")      //Las pixelii sa cada
                ),
                new SleepAction(0.1),
                    /*new ParallelAction(
                            Robot.hook1("close"),
                            Robot.hook2("close")
                    ),*/
                new SleepAction(0.1),
                Robot.openOuttake("close"),
                new SleepAction(0.1),
                Robot.turnOuttake("noTurn"),
                new SleepAction(0.1),
                Robot.misumHeight("GROUND"),

                ParkingLeft
        ));

        while(opModeIsActive() && !isStopRequested()){
            drive.updatePoseEstimate();
            telemetry.addData("x :",drive.pose.position.x);
            telemetry.addData("y :",drive.pose.position.y);
            telemetry.addData("Current Pose :",cPose);
        }
    }
}