package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class newBotTest extends OpMode {
    DcMotor FLM, FRM, BRM, BLM, liftMotor;
    Servo clawServo, forkServo;
    double driveSpeed = 1.0, liftPowerUp = 0.8, liftPowerDown = 0.4;
    public static double clawClose = 0.93, clawOpen = 0.2;
    public static int highPos = 1980, midPos = 1410, lowPos = 853, grabPos = 0;
    public static double forkUp = 1.0, forkDown = 0.0;
    @Override
    public void init() {
        FLM = hardwareMap.get(DcMotor.class, "FLM02");
        FRM = hardwareMap.get(DcMotor.class, "FRM03");
        BRM = hardwareMap.get(DcMotor.class, "BRM01");
        BLM = hardwareMap.get(DcMotor.class, "BLM00");

        liftMotor = hardwareMap.get(DcMotor.class, "SLM10");

        clawServo = hardwareMap.get(Servo.class, "CGS10");
        forkServo = hardwareMap.get(Servo.class, "JSS11");

        FLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        FLM.setDirection(DcMotorSimple.Direction.REVERSE);
        BLM.setDirection(DcMotorSimple.Direction.REVERSE);

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void loop() {
//        liftMotor.setPower(-gamepad1.left_stick_y);
//        forkServo.setPosition(gamepad2.right_stick_y);
//
        double y = -gamepad1.left_stick_y * driveSpeed;
        double x = gamepad1.left_stick_x * driveSpeed;
        double rx = gamepad1.right_stick_x * driveSpeed;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        FLM.setPower(frontLeftPower);
        BLM.setPower(backLeftPower);
        FRM.setPower(frontRightPower);
        BRM.setPower(backRightPower);

        if(gamepad1.y){
            liftMotor.setTargetPosition(highPos);
        } else if(gamepad1.x){
            liftMotor.setTargetPosition(midPos);
        } else if(gamepad1.b){
            liftMotor.setTargetPosition(lowPos);
        } else if(gamepad1.a){
            liftMotor.setTargetPosition(grabPos);
        }
        //TODO: Cut power to lift when its below a certain encoder value to let it coast down at the bottom
        liftMotor.setPower(liftPowerUp);

        if(gamepad1.left_bumper){
            clawServo.setPosition(clawClose);
        } else if (gamepad1.right_bumper){
            clawServo.setPosition(clawOpen);
        }

        if(gamepad1.dpad_up && clawServo.getPosition() == clawClose){
            forkServo.setPosition(forkUp);
        } else if(gamepad1.dpad_down && clawServo.getPosition() == clawClose){
            forkServo.setPosition(forkDown);
        }
        telemetry.addData("liftPos", liftMotor.getCurrentPosition());
    }
}