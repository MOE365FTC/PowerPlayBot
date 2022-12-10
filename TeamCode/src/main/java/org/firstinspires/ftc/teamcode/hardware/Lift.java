package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Lift {
    HardwareMap hardwareMap;
    Gamepad gamepad1;

    DcMotor liftMotor;
    Servo fourBarServo;
    int high = 600, mid = 400, low = 200, floor = 10; //lift extension
    double upPos = 1.0, straightPos = 0.5, downPos = 0.0;
    double liftPower;
    public Lift(HardwareMap hardwareMap, Gamepad gamepad1){
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;

        liftMotor = hardwareMap.get(DcMotor.class, "SLM");
        fourBarServo = hardwareMap.get(Servo.class, "FBS");

        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void actuate(){
        if(gamepad1.y){
            liftMotor.setTargetPosition(high);
            liftMotor.setPower(liftPower);
            fourBarServo.setPosition(upPos);
        } else if (gamepad1.x){
            liftMotor.setTargetPosition(mid);
            liftMotor.setPower(liftPower);
            fourBarServo.setPosition(upPos);
        } else if (gamepad1.b){
            liftMotor.setTargetPosition(low);
            liftMotor.setPower(liftPower);
            fourBarServo.setPosition(upPos);
        } else if (gamepad1.a){
            liftMotor.setTargetPosition(floor);
            liftMotor.setPower(liftPower);
            fourBarServo.setPosition(downPos);
        } else if (gamepad1.right_stick_button){
            liftMotor.setTargetPosition(floor);
            liftMotor.setPower(liftPower);
            fourBarServo.setPosition(straightPos);
        }
    }

    public void autonActuate(autonLiftPos pos) {
        switch (pos) {
            case HIGH:
                liftMotor.setTargetPosition(high);
                liftMotor.setPower(liftPower);
                break;
            case MID:
                liftMotor.setTargetPosition(mid);
                liftMotor.setPower(liftPower);
                break;
            case LOW:
                liftMotor.setTargetPosition(low);
                liftMotor.setPower(liftPower);
                break;
            case HIGH_GRAB:
                liftMotor.setTargetPosition(floor);
                fourBarServo.setPosition(straightPos);
                liftMotor.setPower(liftPower);
                break;
            case LOW_GRAB:
                liftMotor.setTargetPosition(floor);
                fourBarServo.setPosition(downPos);
                liftMotor.setPower(liftPower);
                break;
        }
    }

    public enum autonLiftPos {
        HIGH,
        MID,
        LOW,
        HIGH_GRAB,
        LOW_GRAB,
    }

    public int getLiftTicks() {
        return liftMotor.getCurrentPosition();
    }

    public double getFourBarTicks() {
        return fourBarServo.getPosition();
    }

    public void nudgeFourBar() {
        fourBarServo.setPosition(getFourBarTicks() + 0.1);
    }
}
