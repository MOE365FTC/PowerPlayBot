package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Turret {
    DcMotor turretMotor;

    IMU imu;
    Gamepad gamepad1;
    public Turret(HardwareMap hardwareMap, IMU imu, Gamepad gamepad1){
        this.gamepad1 = gamepad1;
        this.imu = imu;

        turretMotor = hardwareMap.get(DcMotor.class, "TRM");
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void actuate(){
        if(gamepad1.dpad_left){
            turretMotor.setPower(-0.6);
        } else if (gamepad1.dpad_right){
            turretMotor.setPower(0.6);
        } else {
            turretMotor.setPower(0.0);
        }
    }
}
