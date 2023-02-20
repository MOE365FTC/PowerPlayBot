package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Lift {
    HardwareMap hardwareMap;
    Gamepad gamepad2;

    public DcMotor liftMotorL;
    public DcMotor liftMotorR;
    Servo fourBarServo;
    int high = 1440, mid = 860, low = 350, floor = 15; //lift extension
    double upPos = 0.1, straightPos = 0.7, downPos = 0.7;
    double liftPower = 0.4;
    public Lift(HardwareMap hardwareMap, Gamepad gamepad2){
        this.hardwareMap = hardwareMap;
        this.gamepad2 = gamepad2;

        liftMotorL = hardwareMap.get(DcMotor.class, "LLM00");
        liftMotorR = hardwareMap.get(DcMotor.class, "RLM01");
        fourBarServo = hardwareMap.get(Servo.class, "FBS00");

        liftMotorR.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotorL.setDirection(DcMotorSimple.Direction.REVERSE);

        liftMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
        } else if (gamepad2.a) {
            liftMotorL.setTargetPosition(floor);
            liftMotorL.setPower(liftPower);
            liftMotorR.setTargetPosition(floor);
            liftMotorR.setPower(liftPower);
            fourBarServo.setPosition(straightPos);
        } else if (gamepad2.right_stick_button){
            liftMotorL.setTargetPosition(floor);
            liftMotorL.setPower(liftPower);
//            liftMotorR.setTargetPosition(floor);
//            liftMotorR.setPower(liftPower);
            fourBarServo.setPosition(straightPos);
        }
    }

//    public void autonActuate(autonLiftPos pos) {
//        switch (pos) {
//            case HIGH:
//                liftMotorL.setTargetPosition(high);
//                liftMotorL.setPower(liftPower);
////                liftMotorR.setTargetPosition(high);
////                liftMotorR.setPower(liftPower);
//                break;
//            case MID:
//                liftMotorL.setTargetPosition(mid);
//                liftMotorL.setPower(liftPower);
////                liftMotorR.setTargetPosition(mid);
////                liftMotorR.setPower(liftPower);
//                break;
//            case LOW:
//                liftMotorL.setTargetPosition(low);
//                liftMotorL.setPower(liftPower);
////                liftMotorR.setTargetPosition(low);
////                liftMotorR.setPower(liftPower);
//                break;
//            case HIGH_GRAB:
//                liftMotorL.setTargetPosition(floor);
//                liftMotorL.setPower(liftPower);
////                liftMotorR.setTargetPosition(floor);
////                liftMotorR.setPower(liftPower);
//                fourBarServo.setPosition(straightPos);
//                break;
//            case LOW_GRAB:
//                liftMotorL.setTargetPosition(floor);
//                liftMotorL.setPower(liftPower);
////                liftMotorR.setTargetPosition(floor);
////                liftMotorR.setPower(liftPower);
//                fourBarServo.setPosition(downPos);
//                break;
//        }
//    }

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

    public void startFourBarPos(){
        fourBarServo.setPosition(upPos);
    }

    public void manualLift(){
        if(gamepad2.right_trigger > 0.1){
            liftMotorL.setTargetPosition((int) (liftMotorL.getCurrentPosition() + 60));
            liftMotorR.setTargetPosition((int) (liftMotorL.getCurrentPosition() + 60));
            liftMotorL.setPower(liftPower);
        } else if(gamepad2.left_trigger > 0.1){
            liftMotorL.setTargetPosition((int) (liftMotorL.getCurrentPosition() - 60));
            liftMotorR.setTargetPosition((int) (liftMotorL.getCurrentPosition() - 60));
            liftMotorL.setPower(liftPower);
        }
    }
}