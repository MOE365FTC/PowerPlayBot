package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class TempFieldCentricDrive extends LinearOpMode {
    double angle;
    double autoTurnPower;
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("FLM");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("BLM");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("FRM");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("BRM");

        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double botHeading = -imu.getAngularOrientation().firstAngle;

            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            motorFrontLeft.setPower(frontLeftPower * 0.6);
            motorBackLeft.setPower(backLeftPower * 0.6);
            motorFrontRight.setPower(frontRightPower * 0.6);
            motorBackRight.setPower(backRightPower * 0.6);
            angle = Math.toDegrees(imu.getAngularOrientation().firstAngle) + 180.0;
            telemetry.addData("angle", angle);
            telemetry.update();
            if(gamepad1.a){
                autoTurnPower = Math.max(Math.min(angle%90, 90-angle%90)/80, 0.2);
                if(angle % 90 >= 45){
                    motorFrontLeft.setPower(-autoTurnPower);
                    motorFrontLeft.setPower(-autoTurnPower);
                    motorBackRight.setPower(autoTurnPower);
                    motorFrontRight.setPower(autoTurnPower);
                } else if(angle % 90 < 45){
                    motorFrontLeft.setPower(autoTurnPower);
                    motorFrontLeft.setPower(autoTurnPower);
                    motorBackRight.setPower(-autoTurnPower);
                    motorFrontRight.setPower(-autoTurnPower);
                }
            }
        }
    }
}
