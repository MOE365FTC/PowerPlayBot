package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp
public class NewClawTestPP extends OpMode {
    Servo clawRight, clawLeft;

    double clawOpenPosR = 0.0;
    double clawClosePosR = 0.5;
    double clawOpenPosL = 0.8;
    double clawClosePosL = 0.0;

    int armPos = 0;
    DcMotor frontLeft, backLeft, frontRight, backRight, armMotor;
    @Override
    public void init() {
        clawRight = hardwareMap.get(Servo.class, "TSS01");
        clawLeft = hardwareMap.get(Servo.class, "DGS00");
        frontLeft = hardwareMap.get(DcMotor.class, "FLM02");
        frontRight = hardwareMap.get(DcMotor.class, "FRM12");
        backLeft = hardwareMap.get(DcMotor.class, "BLM03");
        backRight = hardwareMap.get(DcMotor.class, "BRM13");
        armMotor = hardwareMap.get(DcMotor.class, "OAM01");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void loop() {
        if(gamepad1.right_trigger > 0.9) {
            clawRight.setPosition(clawClosePosR);
            clawLeft.setPosition(clawClosePosL);
        } else {
            clawRight.setPosition(clawOpenPosR);
            clawLeft.setPosition(clawOpenPosL);
        }

        if(gamepad1.dpad_up){
            armMotor.setTargetPosition(armMotor.getCurrentPosition() + 10);
        } else if(gamepad1.dpad_down){
            armMotor.setTargetPosition(armMotor.getCurrentPosition() - 10);
        }
        armMotor.setPower(0.6);
        frontLeft.setPower(-gamepad1.left_stick_y);
        backLeft.setPower(-gamepad1.left_stick_y);
        backRight.setPower(-gamepad1.right_stick_y);
        frontRight.setPower(-gamepad1.right_stick_y);
    }
}
