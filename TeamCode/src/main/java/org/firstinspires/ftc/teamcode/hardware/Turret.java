package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {
    DcMotor turretMotor;
    IMU imu;
    Gamepad gamepad1;

    int ticksPerDegree = 100; //temp value

    public Turret(HardwareMap hardwareMap, IMU imu, Gamepad gamepad1) {
        this.gamepad1 = gamepad1;
        this.imu = imu;

        turretMotor = hardwareMap.get(DcMotor.class, "TRM");
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void actuate() {
        if (gamepad1.dpad_up && gamepad1.dpad_right) {
            //45 deg
            turnToDegree(45);
        } else if (gamepad1.dpad_right && gamepad1.dpad_down) {
            //135 deg
            turnToDegree(135);
        } else if (gamepad1.dpad_down && gamepad1.dpad_left) {
            //225 deg
            turnToDegree(225);
        } else if (gamepad1.dpad_left && gamepad1.dpad_up) {
            //315 deg
            turnToDegree(315);
        } else if (gamepad1.dpad_up) {
            //0 deg
            turnToDegree(0);
        } else if (gamepad1.dpad_right) {
            //90 deg
            turnToDegree(90);
        } else if (gamepad1.dpad_down) {
            //180 deg
            turnToDegree(180);
        } else if (gamepad1.dpad_left) {
            //270 deg
            turnToDegree(270);
        }
        //        if(gamepad1.dpad_left){
//            turretMotor.setPower(-0.6);
//        } else if (gamepad1.dpad_right){
//            turretMotor.setPower(0.6);
//        } else {
//            turretMotor.setPower(0.0);
//        }
    }

    public void turnToDegree(int turnDegree) {
        double correctedDegrees = ((turnDegree + imu.getHeadingFirstAngle()) % 360) * ticksPerDegree; //field-centric angle
        //finish this

    }
}
