package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TestingHwMap {

    public GamepadEx gamepad1Ex, gamepad2Ex;

    public TestingHwMap(HardwareMap hwMap){
        this.ahwMap=hwMap;
    }
    HardwareMap ahwMap;

    public Object initComponent(Class a, HardwareMap hwMap, String name){
        hwMap=ahwMap;
        return hwMap.get(a, name);
    }
    public void removeComponent(HardwareDevice a, HardwareMap hwMap, String name){
        hwMap=ahwMap;
        hwMap.remove(name, a);
    }

    public void gamepadInit(Gamepad gmpd1, Gamepad gmpd2){
        gamepad1Ex = new GamepadEx(gmpd1);
        gamepad2Ex = new GamepadEx(gmpd2);
    }

}
