package org.firstinspires.ftc.teamcode.Mara;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.HardwareMapping;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

import java.util.ArrayList;
import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOpMara")
public class TeleOpMara extends LinearOpMode {

    HardwareMapping Robot=new HardwareMapping();
    HardwareMapping.Intake intake = Robot.new Intake();
    HardwareMapping.Outtake outtake = Robot.new Outtake();

    private List<Action> runningActions = new ArrayList<>();

    //HardwareMapping hw=new HardwareMapping();
    private MecanumDrive drive;
    @Override
    public void runOpMode() throws InterruptedException {

        Robot.init(hardwareMap);
        Robot.gamepadInit(gamepad1,gamepad2);
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        double TriggerSlowdown=gamepad2.right_trigger,heading=180;

        waitForStart();

        while(opModeIsActive()){

            if(isStopRequested())return;

            if(TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
                drive.setDrivePowers(new PoseVelocity2d(   //miscarea de baza a robotului
                        new Vector2d(
                                -gamepad2.left_stick_y,
                                -gamepad2.left_stick_x
                        ),
                        -gamepad2.right_stick_x
                ));
            }

            //gamepad1

            if(gamepad1.x){
                runningActions.add(intake.powerOn());
            }
            if(gamepad2.x) {
                runningActions.add(intake.reverse()); //adaugat nou
            }
            if(gamepad1.b){

            }
        }

    }
}