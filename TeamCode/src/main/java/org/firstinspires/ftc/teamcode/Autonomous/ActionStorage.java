package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;

import org.firstinspires.ftc.teamcode.HardwareMapping;
import org.firstinspires.ftc.teamcode.Variables.DefVal;

public class ActionStorage {
    HardwareMapping.Intake intake;
    HardwareMapping.Outtake outtake;
    public ActionStorage(HardwareMapping.Intake intake, HardwareMapping.Outtake outtake){
        this.intake = intake;
        this.outtake = outtake;
    }
    public Action pixelToLow = new ParallelAction(
            new SequentialAction(
                    outtake.runToPosition("low"),
                    new ParallelAction(
                            outtake.pivot(DefVal.pivot60),
                            outtake.roll(DefVal.roll60)
                    ),
                    //outtake.yaw(DefVal.yaw90),
                    outtake.latch("open")
            ),
            intake.sensingOff()
    ),
            pixelToMiddle = new ParallelAction(
                    new SequentialAction(
                            outtake.runToPosition("middle"),
                            new ParallelAction(
                                    outtake.pivot(DefVal.pivot60),
                                    outtake.roll(DefVal.roll60)
                            ),
                            //outtake.yaw(DefVal.yaw90),
                            outtake.latch("open")
                    ),
                    intake.sensingOff()
            ),
            pixelToGround = new SequentialAction(
                    new ParallelAction(
                            outtake.latch("closed")
                            //outtake.yaw(DefVal.yaw0)
                    ),
                    new ParallelAction(
                            outtake.pivot(DefVal.pivot0),
                            outtake.roll(DefVal.roll0)
                    ),
                    outtake.runToPosition("ground")
            );
}
