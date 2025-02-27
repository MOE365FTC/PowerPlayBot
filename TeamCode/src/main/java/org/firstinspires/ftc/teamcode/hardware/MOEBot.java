package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.signalDetector.Vision;

public class MOEBot {
    public Chassis chassis;
    public Claw claw;
    public IMU imu;
    public Vision vision;
    public Lift lift;

    //TeleOp Constructor
    public MOEBot(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        imu = new IMU(hardwareMap);
        chassis = new Chassis(hardwareMap, imu, gamepad1);
        claw = new Claw(hardwareMap, gamepad1, gamepad2);
        lift = new Lift(hardwareMap, gamepad1, gamepad2);
        vision = new Vision(hardwareMap);
    }

    //Autonomous Constructor
//    public MOEBot(HardwareMap hardwareMap, Gamepad gamepad1) {
//        imu = new IMU(hardwareMap);
//        chassis = new Chassis(hardwareMap, imu, gamepad1);
//        claw = new Claw();
//        turret = new Turret(hardwareMap, imu, gamepad1);
//        vision = new Vision(hardwareMap);
//    }
}