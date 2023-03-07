package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Chassis {
    public DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;

    Gamepad gamepad1;
    LinearOpMode opMode;
    IMU imu;
    double headingOffset = 0;
    double driveSpeed = 0.6, scaleFactor;

    //Teleop Constructor
    public Chassis(HardwareMap hardwareMap, IMU imu, Gamepad gamepad1){
     this.gamepad1 = gamepad1;
     this.imu = imu;

     frontLeftMotor = hardwareMap.get(DcMotor.class, "FLM10");
     backLeftMotor = hardwareMap.get(DcMotor.class, "BLM12");
     frontRightMotor = hardwareMap.get(DcMotor.class, "FRM11");
     backRightMotor = hardwareMap.get(DcMotor.class, "BRM13");

//     backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//     frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
     frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
     backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
     frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
     backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void fieldCentricDrive(){
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double botHeading = Math.toRadians(-imu.getHeadingFirstAngle()) - headingOffset;

        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        scaleFactor = driveSpeed + gamepad1.right_trigger * (1 - driveSpeed);
        frontLeftMotor.setPower(frontLeftPower * scaleFactor);
        backLeftMotor.setPower(backLeftPower * scaleFactor);
        frontRightMotor.setPower(frontRightPower * scaleFactor);
        backRightMotor.setPower(backRightPower * scaleFactor);

        if(gamepad1.right_stick_button){
            headingOffset = -imu.getHeadingFirstAngle();
        }
    }
}
