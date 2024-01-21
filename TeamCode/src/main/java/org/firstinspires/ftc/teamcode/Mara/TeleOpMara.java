package org.firstinspires.ftc.teamcode.Mara;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Mara.HardwareMapp;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

import java.util.ArrayList;
import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOpMara")
public class TeleOpMara extends LinearOpMode {
    HardwareMapp Robot=new HardwareMapp();
    private List<Action> runningActions = new ArrayList<>();
    private boolean isXPressed = false;
    private boolean isYPressed=false;
    private boolean isAPressed=false;
    private boolean isBPressed=false;
    private boolean isAPressedforOpenOuttake=false;
    private int isBumperPressed=7;
    public DcMotorEx misumMotorLeft;  //motor pentru misum-ul stang
    public DcMotorEx misumMotorRight;  //motor pentru misum-ul drept
    HardwareMap HW=null;
    public static double LiftGROUND = 0;

    //HardwareMapping hw=new HardwareMapping();
    private MecanumDrive drive;

    /**
     *  DRIVER 1
     *   X         - Power on/off INTAKE                 //Gata
     *   Y         - Engage hooks on/off                 //Gata
     *   B         - Hanging                             //Gata
     *   A         - Rotate outtake 90 degrees           //Gata
     *   Left/Right stick - Base controls
     *   Right stick press down - Launch plane           //Gata
     *   DPAD left     - Lift ground                     //Gata
     *   DPAD down     - Lift 1st level                  //Gata
     *   DPAD right    - Lift 2nd level                  //Gata
     *   DPAD up       - Lift 3rd level                  //Gata
     *   LEFT/RIGHT BUMPER - Change intake angle         //Gata
     *
     *   DRIVER 2
     *   X         - Power on/off INTAKE                 //Gata
     *   Y         - Engage hooks on/off                 //Gata
     *   B         - Reverse intake                      //Gata
     *   A         - Open outtake                        //Gata
     *   Left stick Y - manual slide control
     *   Left stick X - manual outtake pitch (keep 60 degree angle (in progress))
     *   Right stick press down - Launch plane           //Gata
     *   DPAD left     - Lift ground                     //Gata
     *   DPAD down     - Lift 1st level                  //Gata
     *   DPAD right    - Lift 2nd level                  //Gata
     *   DPAD up       - Lift 3rd level                  //Gata
     *   LEFT/RIGHT BUMPER - Change intake angle         //Gata
     *   BACK          - Change movement mode
     *   START         - Change HEADING_LOCK target 0/180
     *
     *       _=====_                               _=====_
     *      / _____ \                             / _____ \
     *    +.-'_____'-.---------------------------.-'_____'-.+
     *   /   |     |  '.                       .'  |  _  |   \
     *  / ___| /|\ |___ \    BACK     START   / ___| (Y) |___ \
     * / |      |      | ;                   ; |              | ;
     * | | <---   ---> | |                   | |(X)       (B) | |
     * | |___   |   ___| ;  MODE             ; |___        ___| ;
     * |\    | \|/ |    /  _              _   \    | (A) |    /|
     * | \   |_____|  .','" "',        ,'" "', '.  |_____|  .' |
     * |  '-.______.-' /       \      /       \  '-._____.-'   |
     * |               |       |------|       |                |
     * |              /\       /      \       /\               |
     * |             /  '.___.'        '.___.'  \              |
     * |            /                            \             |
     *  \          /                              \           /
     *   \________/                                \_________/
     */

    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        double TriggerSlowdown=gamepad2.right_trigger,heading=180;
        Robot.init(hardwareMap);
        Robot.gamepadInit(gamepad1, gamepad2);
        misumMotorLeft=HW.get(DcMotorEx.class,"misumMotorLeft");
        misumMotorRight=HW.get(DcMotorEx.class,"misumMotorRight");

        waitForStart();

        while(opModeIsActive()){

            if(isStopRequested())return;
            TelemetryPacket packet = new TelemetryPacket();

            if(TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
                drive.setDrivePowers(new PoseVelocity2d(   //miscarea de baza a robotului
                        new Vector2d(
                                -gamepad2.left_stick_y,
                                -gamepad2.left_stick_x
                        ),
                        -gamepad2.right_stick_x
                ));
            }

            //gamepad1 && gamepad2

            if (Robot.gamepad1Ex.wasJustPressed(GamepadKeys.Button.X) || Robot.gamepad2Ex.wasJustPressed(GamepadKeys.Button.X)) {
                if (!isXPressed) {                                 //Directia intake-ului in functie de cati pixeli exista
                    isXPressed = true;
                    runningActions.add(new SequentialAction(
                            Robot.intakeRoller("in")
                    ));
                } else {
                    isXPressed = false;
                    runningActions.add(new SequentialAction(
                            Robot.intakeRoller("out ")
                    ));
                }
            }
            if(Robot.gamepad1Ex.wasJustPressed(GamepadKeys.Button.Y) || Robot.gamepad2Ex.wasJustPressed(GamepadKeys.Button.Y)){
                if(!isYPressed){                                   //Deschis/inchis hookul pentru pixeli
                    isYPressed=true;
                    runningActions.add(new ParallelAction(
                            Robot.bottomHook("close"),
                            Robot.upperHook("close")
                    ));
                } else {
                    isYPressed=false;
                    runningActions.add(new ParallelAction(
                            Robot.bottomHook("open"),
                            Robot.upperHook("open")
                    ));
                }
            }
            if(Robot.gamepad1Ex.wasJustPressed(GamepadKeys.Button.B)){   //Pentru hang
                runningActions.add(Robot.hang("up"));
            }
            /*if(Robot.gamepad1Ex.wasJustPressed(GamepadKeys.Button.A) && misumMotorLeft.equals(LiftGROUND) && misumMotorRight.equals(LiftGROUND)){
                if(!isAPressed){                                           //Turn outake la 90 de grade
                    isAPressed=true;
                    runningActions.add(Robot.turn90Outtake("turn"));
                } else {
                    isAPressed=false;
                    runningActions.add(Robot.turn90Outtake("noTurn"));
                }
            }*/
            /*if(Robot.gamepad2Ex.wasJustPressed(GamepadKeys.Button.A)){   //Open/Close outake
                if(!isAPressedforOpenOuttake){
                    isAPressedforOpenOuttake=true;
                    runningActions.add(Robot.openOuttake("open"));
                } else {
                    isAPressedforOpenOuttake=false;
                    runningActions.add(Robot.openOuttake("close"));
                }
            }*/
            if(Robot.gamepad1Ex.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON) || Robot.gamepad2Ex.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)){
                runningActions.add(Robot.launchPlane());           //Launch plane
            }
            if(Robot.gamepad1Ex.wasJustPressed(GamepadKeys.Button.DPAD_LEFT) || Robot.gamepad2Ex.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)){
                runningActions.add(Robot.misumHeight("GROUND"));    //Misum ground
            }
            if(Robot.gamepad1Ex.wasJustPressed(GamepadKeys.Button.DPAD_DOWN) || Robot.gamepad2Ex.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
                runningActions.add(Robot.misumHeight("LOW"));       //Misum low
            }
            if(Robot.gamepad1Ex.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT) || Robot.gamepad2Ex.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)){
                runningActions.add(Robot.misumHeight("MIDDLE"));    //Misum middle
            }
            if(Robot.gamepad1Ex.wasJustPressed(GamepadKeys.Button.DPAD_UP) || Robot.gamepad2Ex.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
                runningActions.add(Robot.misumHeight("HIGH"));      //Misum high
            }
            if(Robot.gamepad2Ex.wasJustPressed(GamepadKeys.Button.B)){     //Maturice directie in/out
                if(!isBPressed){
                    isBPressed=true;
                    runningActions.add(Robot.maturiceOpen_Close("in"));
                } else {
                    isBPressed=false;
                    runningActions.add(Robot.maturiceOpen_Close("out"));
                }
            }
            if(Robot.gamepad2Ex.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER) || Robot.gamepad2Ex.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)){
                isBumperPressed--;                                          //Level maturice(1/2/3/4/5/6)
                if(isBumperPressed==1){
                    isBumperPressed=7;
                    runningActions.add(Robot.maturiceLevel("Level1"));
                }
                if(isBumperPressed==2){
                    runningActions.add(Robot.maturiceLevel("Level2"));
                }
                if(isBumperPressed==3){
                    runningActions.add(Robot.maturiceLevel("Level3"));
                }
                if(isBumperPressed==4){
                    runningActions.add(Robot.maturiceLevel("Level4"));
                }
                if(isBumperPressed==5){
                    runningActions.add(Robot.maturiceLevel("Level5"));
                }
                if(isBumperPressed==6){
                    runningActions.add(Robot.maturiceLevel("Level6"));
                }
            }
            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;
        }
    }
}