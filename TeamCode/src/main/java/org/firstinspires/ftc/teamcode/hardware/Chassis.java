package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Chassis {
    DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;

    Gamepad gamepad1;
    LinearOpMode opMode;
    IMU imu;

    double powerScalar = 0.6;

    //Teleop Constructor
    public Chassis(HardwareMap hardwareMap, IMU imu, Gamepad gamepad1){
     this.gamepad1 = gamepad1;
     this.imu = imu;

     frontLeftMotor = hardwareMap.get(DcMotor.class, "FLM");
     backLeftMotor = hardwareMap.get(DcMotor.class, "BLM");
     frontRightMotor = hardwareMap.get(DcMotor.class, "FRM");
     backRightMotor = hardwareMap.get(DcMotor.class, "BRM");

     frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
     backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

     frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
     backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
     frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
     backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void FieldCentricDrive(){
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double botHeading = -imu.getHeadingFirstAngle();

        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower * powerScalar);
        backLeftMotor.setPower(backLeftPower * powerScalar);
        frontRightMotor.setPower(frontRightPower * powerScalar);
        backRightMotor.setPower(backRightPower * powerScalar);
    }
}
