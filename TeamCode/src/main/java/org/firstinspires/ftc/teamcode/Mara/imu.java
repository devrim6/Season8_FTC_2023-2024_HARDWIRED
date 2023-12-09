package org.firstinspires.ftc.teamcode.Mara;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class imu {
    private static final double TILT_THRESHOLD = 15.0;
    //private MecanumDrive MDrive=new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
    public IMU imu;
    public imu(IMU imu) {
        this.imu = imu;
    }
    DcMotor leftFront=hardwareMap.get(DcMotor.class,"LF");
    DcMotor leftBack=hardwareMap.get(DcMotor.class,"LB");
    DcMotor rightFront=hardwareMap.get(DcMotor.class,"RF");
    DcMotor rightBack=hardwareMap.get(DcMotor.class,"RB");

    public boolean isRobotTilted() {
        double pitchAngle = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle;
        //return Math.abs(pitchAngle) > TILT_THRESHOLD;

        if(Math.abs(pitchAngle)>TILT_THRESHOLD){

            telemetry.addData("Status", "Robotul poate sta pe 2 roti");
            telemetry.addData("PitchAngle", pitchAngle);

            double yawAngle = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            if(yawAngle>0){
                //Ii inclinat in fata
                //folosesc motoarele din fata.Ii dau viteza mai mare in fata.Cred;(
                leftFront.setPower(1);
                rightFront.setPower(1);
            }
            else{
                //Ii inclinat pe spate
                //folosesc motoarele din spate.Ii dau viteza mai mica in spate.Cred:)
                leftBack.setPower(-0.5);
                rightBack.setPower(-0.5);
            }

        }
        else{
            //Este ok
            telemetry.addData("Status","Robotul e ok");
        }
        return false;
    }
}