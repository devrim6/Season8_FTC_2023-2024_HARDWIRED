package org.firstinspires.ftc.teamcode.AutonomieMara;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Mara.HardwareMapp;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.cameraStuff.cameraHW;

@Autonomous(name = "AutoTestRedStack")
public class AutoTestRedStack extends LinearOpMode {
    //Robotul de langa stack merge pe sus, cel de langa backboard pe jos
    //Nu ia pixel prima oare cand il lasa pe cel mov jos
    HardwareMapp Robot = new HardwareMapp();
    private MecanumDrive drive;
    private Pose2d cPose;
    cameraHW camera = new cameraHW();
    String elementPosition = "middle";
    @Override
    public void runOpMode() throws InterruptedException {

        Robot.init(hardwareMap);
        camera.initTeamPropCamera("RED");
        elementPosition=camera.isPointInsideRect();
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

        cPose=firstPose;

        Action TrajMiddleLane = drive.actionBuilder(cPose)
                .splineToLinearHeading(new Pose2d(-35,-33,Math.toRadians(90)),Math.toRadians(90))
                //nsh daca trebe si astea
                //.waitSeconds(0.7)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-35,-42,Math.toRadians(90)),Math.toRadians(90))
                .build();

        cPose=firstPose;

        Action TrajLeftLane = drive.actionBuilder(cPose)
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(-39,-36,Math.toRadians(120)),Math.toRadians(90))
                //nsh daca trebe si asta
                //.waitSeconds(0.2)
                //.setReversed(true)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-37,-45,Math.toRadians(90)),Math.toRadians(90))
                .setReversed(false)
                .build();

        cPose=RightLane; //nsh daca trebuie luata pozitia separat dupa fiecare traiectorie in functie de ce pica

        Action TrajLeftStack = drive.actionBuilder(cPose)  //Traiectorie pana la Stackul din stanga
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-56.45,-36,Math.toRadians(0)),Math.toRadians(90))
                /*.afterDisp(1, new SequentialAction(  //1 inch = 2,54 cm
                        new ParallelAction(
                                Robot.bottomHook("open"),
                                Robot.intakeRoller("in"),
                                Robot.maturiceOpen_Close("open"),
                                Robot.maturiceLevel("Level5")
                        )
                ))*/
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
                .afterDisp(1,new SequentialAction(  //1 inch = 2,54 cm, (3 1/2), 1=23 inch (a trecut de truss)
                        Robot.misumHeight("MIDDLE"),
                        //new SleepAction(0.1),
                        new ParallelAction(
                                Robot.turnOuttakeUp_Down("up"),
                                Robot.outtakeRoll("backboard")  //daca nu merge asa fac una dupa alta actiunile
                        )
                        //Robot.turn90Outtake("turn")
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
                                Robot.intakeRoller("in"),
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
                                Robot.intakeRoller("in"),
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
                                Robot.intakeRoller("in"),
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
                                Robot.intakeRoller("in"),
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
                                Robot.intakeRoller("in"),
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
                                Robot.intakeRoller("in"),
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
                        //new SleepAction(0.1),
                        new ParallelAction(
                                Robot.turnOuttakeUp_Down("up"),
                                Robot.outtakeRoll("backboard")
                        )
                        //Robot.turn90Outtake("turn")
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
                                Robot.bottomHook("open"),
                                Robot.upperHook("open")
                        ),
                        //new SleepAction(0.2),
                        //Robot.openOuttake("close"),
                        //Robot.turn90Outtake("noTurn"),
                        new ParallelAction(
                                Robot.turnOuttakeUp_Down("down"),
                                Robot.outtakeRoll("ground")
                        ),
                        //new SleepAction(0.1),
                        new ParallelAction(
                                Robot.maturiceOpen_Close("off"),
                                Robot.intakeRoller("off"),
                                Robot.maturiceLevel("Level1"),
                                Robot.misumHeight("GROUND")
                        )
                ))
                .build();

        camera.initTeamPropCamera("RED");
        elementPosition=camera.isPointInsideRect();

        waitForStart();

        switch (elementPosition){
            case "Middle":  //daca ii middle,atunci merg la linia din mijloc, dupa merg la backboard mijloc, apoi depinde sus/jos
                Actions.runBlocking(new SequentialAction(
                        TrajMiddleLane
                ));

            case "Left":    //daca ii left,atunci merg la linia din stanga, dupa merg la backboard stanga, apoi depinde sus/jos
                Actions.runBlocking(new SequentialAction(
                        TrajLeftLane
                ));

            case "Right":   //daca ii right,atunci merg la linia din dreapta, dupa merg la backboard dreapta, apoi depinde sus/jos
                Actions.runBlocking(new SequentialAction(
                        TrajRightLane
                ));

        }

        Actions.runBlocking(new SequentialAction( //Face actiunile una dupa cealalta
                Robot.upperHook("close"), //pixelul din cuva
                Robot.bottomHook("close"),  //Pixelul de jos (in cel de sus este deja un pixel)

                TrajLeftStack,
                //new SleepAction(0.7),    // Astept sa ia pixel-poate mai mult trebuie sa stea?
                /*new ParallelAction(
                        Robot.maturiceOpen_Close("off"),
                        Robot.intakeRoller("off")   //Daca ia prea mult timp bag urmatoarea traiectorie in ParallelAction
                ),*/

                TrajToBackboardRight,
                //Robot.openOuttake("open"),
                //new SleepAction(0.1),
                new ParallelAction(
                        Robot.upperHook("open"),
                        Robot.bottomHook("open")      //Las pixelii sa cada
                ),
                //new SleepAction(0.1),
                    /*new ParallelAction(
                            Robot.hook1("close"),
                            Robot.hook2("close")
                    ),*/                                 //Las deschis pentru urmatorii pixeli
                //new SleepAction(0.1),
                //Robot.openOuttake("close"),
                //Robot.turn90Outtake("noTurn"),
                new ParallelAction(
                        Robot.turnOuttakeUp_Down("down"),
                        Robot.outtakeRoll("ground")
                ),
                //new SleepAction(0.1),
                //new SleepAction(0.1),
                Robot.misumHeight("GROUND"),

                TrajRightStackLevel4,                //Level 4 ca sa imi ia primii 2 pixeli din stack ul din dreapta
                new SleepAction(0.7),             //Las timp sa ia pixel
                new ParallelAction(
                        Robot.upperHook("close"),
                        Robot.bottomHook("close")
                ),
                new ParallelAction(
                        Robot.maturiceOpen_Close("off"),
                        Robot.intakeRoller("off")      //Daca ia prea mult timp bag actiunile astea mai sus in parallelAction
                ),

                TrajToBackboardLeft,
                //Robot.openOuttake("open"),
                //new SleepAction(0.1),
                new ParallelAction(
                        Robot.upperHook("open"),
                        Robot.bottomHook("open")      //Las pixelii sa cada
                ),
                //new SleepAction(0.1),
                    /*new ParallelAction(
                            Robot.hook1("close"),
                            Robot.hook2("close")
                    ),*/                                  //Las deschis pentru urmatorii pixeli
                //new SleepAction(0.1),
                //Robot.openOuttake("close"),
                //Robot.turn90Outtake("noTurn"),
                new ParallelAction(
                        Robot.turnOuttakeUp_Down("down"),
                        Robot.outtakeRoll("ground")
                ),
                //new SleepAction(0.1),
                //new SleepAction(0.1),
                Robot.misumHeight("GROUND"),

                TrajRightStackLevel2,                 //Level 2 ca sa imi ia al doilea si al treilea pixel din stack ul din dreapta
                new SleepAction(0.7),              //Las timp sa ia pixel
                new ParallelAction(
                        Robot.upperHook("close"),
                        Robot.bottomHook("close")
                ),
                new ParallelAction(
                        Robot.maturiceOpen_Close("off"),
                        Robot.intakeRoller("off")      //Daca ia prea mult timp bag actiunile astea mai sus in parallelAction
                ),

                TrajToBackboardLeft,
                //Robot.openOuttake("open"),
                //new SleepAction(0.1),
                new ParallelAction(
                        Robot.upperHook("open"),
                        Robot.bottomHook("open")      //Las pixelii sa cada
                ),
                //new SleepAction(0.1),
                    /*new ParallelAction(
                            Robot.hook1("close"),
                            Robot.hook2("close")
                    ),*/
                //new SleepAction(0.1),
                //Robot.openOuttake("close"),
                //Robot.turn90Outtake("noTurn"),
                new ParallelAction(
                        Robot.turnOuttakeUp_Down("down"),
                        Robot.outtakeRoll("ground")
                ),
                //new SleepAction(0.1),
                //new SleepAction(0.1),
                Robot.misumHeight("GROUND"),

                ParkingLeft
        ));

        while(opModeIsActive() && !isStopRequested()){
            telemetry.addData("x :",drive.pose.position.x);
            telemetry.addData("y :",drive.pose.position.y);
            telemetry.addData("Current Pose :",cPose);
            telemetry.update();
            drive.updatePoseEstimate();
        }
    }
}