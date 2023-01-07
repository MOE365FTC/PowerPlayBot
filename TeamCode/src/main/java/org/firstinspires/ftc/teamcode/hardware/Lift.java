package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Lift {
    HardwareMap hardwareMap;
    Gamepad gamepad2;

    DcMotor liftMotorL, liftMotorR;
    Servo fourBarServo;
    int high = 600, mid = 400, low = 200, floor = 10; //lift extension
    double upPos = 1.0, straightPos = 0.5, downPos = 0.0;
    double liftPower = 0.6;
    public Lift(HardwareMap hardwareMap, Gamepad gamepad2){
        this.hardwareMap = hardwareMap;
        this.gamepad2 = gamepad2;

        liftMotorL = hardwareMap.get(DcMotor.class, "LLM");
        liftMotorR = hardwareMap.get(DcMotor.class, "RLM");
        fourBarServo = hardwareMap.get(Servo.class, "FBS");

        liftMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotorL.setTargetPosition(0);
        liftMotorR.setTargetPosition(0);
        liftMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void actuate(){
        if(gamepad2.y){
            liftMotorL.setTargetPosition(high);
            liftMotorL.setPower(liftPower);
            liftMotorR.setTargetPosition(high);
            liftMotorR.setPower(liftPower);
            fourBarServo.setPosition(upPos);
        } else if (gamepad2.x){
            liftMotorL.setTargetPosition(mid);
            liftMotorL.setPower(liftPower);
            liftMotorR.setTargetPosition(mid);
            liftMotorR.setPower(liftPower);
            fourBarServo.setPosition(upPos);
        } else if (gamepad2.b){
            liftMotorL.setTargetPosition(low);
            liftMotorL.setPower(liftPower);
            liftMotorR.setTargetPosition(low);
            liftMotorR.setPower(liftPower);
            fourBarServo.setPosition(upPos);
        } else if (gamepad2.a){
            liftMotorL.setTargetPosition(floor);
            liftMotorL.setPower(liftPower);
            liftMotorR.setTargetPosition(floor);
            liftMotorR.setPower(liftPower);
            fourBarServo.setPosition(downPos);
        } else if (gamepad2.right_stick_button){
            liftMotorL.setTargetPosition(floor);
            liftMotorL.setPower(liftPower);
            liftMotorR.setTargetPosition(floor);
            liftMotorR.setPower(liftPower);
            fourBarServo.setPosition(straightPos);
        }
    }

    public void autonActuate(autonLiftPos pos) {
        switch (pos) {
            case HIGH:
                liftMotorL.setTargetPosition(high);
                liftMotorL.setPower(liftPower);
                liftMotorR.setTargetPosition(high);
                liftMotorR.setPower(liftPower);
                break;
            case MID:
                liftMotorL.setTargetPosition(mid);
                liftMotorL.setPower(liftPower);
                liftMotorR.setTargetPosition(mid);
                liftMotorR.setPower(liftPower);
                break;
            case LOW:
                liftMotorL.setTargetPosition(low);
                liftMotorL.setPower(liftPower);
                liftMotorR.setTargetPosition(low);
                liftMotorR.setPower(liftPower);
                break;
            case HIGH_GRAB:
                liftMotorL.setTargetPosition(floor);
                liftMotorL.setPower(liftPower);
                liftMotorR.setTargetPosition(floor);
                liftMotorR.setPower(liftPower);
                fourBarServo.setPosition(straightPos);
                break;
            case LOW_GRAB:
                liftMotorL.setTargetPosition(floor);
                liftMotorL.setPower(liftPower);
                liftMotorR.setTargetPosition(floor);
                liftMotorR.setPower(liftPower);
                fourBarServo.setPosition(downPos);
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

    public int getLiftTicksL() {
        return liftMotorL.getCurrentPosition();
    }

    public int getLiftTicksR() {
        return liftMotorR.getCurrentPosition();
    }

    public double getFourBarTicks() {
        return fourBarServo.getPosition();
    }

    public void nudgeFourBar() {
        fourBarServo.setPosition(getFourBarTicks() + 0.1);
    }
}
