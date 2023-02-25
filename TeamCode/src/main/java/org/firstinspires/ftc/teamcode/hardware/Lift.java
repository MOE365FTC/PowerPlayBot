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
    int high = 1440, mid = 860, low = 350, floor = 15, stack5 = 280, stack4 = 250, stack3 = 60, stack2 = 30; //lift extension
    double upPos = 0.2, straightPos = 0.8;
    double liftPower = 0.7;
    double autonLiftPower = 0.55;
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
        } else if (gamepad2.dpad_up){
            liftMotorL.setTargetPosition(stack5);
            liftMotorL.setPower(liftPower);
            liftMotorR.setTargetPosition(stack5);
            liftMotorR.setPower(liftPower);
            fourBarServo.setPosition(straightPos);
        }else if (gamepad2.dpad_right){
            liftMotorL.setTargetPosition(stack4);
            liftMotorL.setPower(liftPower);
            liftMotorR.setTargetPosition(stack4);
            liftMotorR.setPower(liftPower);
            fourBarServo.setPosition(straightPos);
        } else if (gamepad2.dpad_left){
            liftMotorL.setTargetPosition(stack3);
            liftMotorL.setPower(liftPower);
            liftMotorR.setTargetPosition(stack3);
            liftMotorR.setPower(liftPower);
            fourBarServo.setPosition(straightPos);
        } else if (gamepad2.dpad_down){
            liftMotorL.setTargetPosition(stack2);
            liftMotorL.setPower(liftPower);
            liftMotorR.setTargetPosition(stack2);
            liftMotorR.setPower(liftPower);
            fourBarServo.setPosition(straightPos);
        }

        if(Math.abs(-gamepad2.left_stick_y) > 0.1 && fourBarServo.getPosition() <= 0.8 && fourBarServo.getPosition() >= 0.2){
                fourBarServo.setPosition(fourBarServo.getPosition() + 0.01 * -Math.signum(gamepad2.left_stick_y));
        }
        if(fourBarServo.getPosition() > 0.8) fourBarServo.setPosition(0.8);
        if(fourBarServo.getPosition() < 0.2) fourBarServo.setPosition(0.2);
    }

    public void autonActuate(autonLiftPos pos) {
        switch (pos) {
            case HIGH:
                liftMotorL.setTargetPosition(high);
                liftMotorL.setPower(autonLiftPower);
                liftMotorR.setTargetPosition(high);
                liftMotorR.setPower(autonLiftPower);
                fourBarServo.setPosition(upPos);
                break;
            case MID:
                liftMotorL.setTargetPosition(mid);
                liftMotorL.setPower(autonLiftPower);
                liftMotorR.setTargetPosition(mid);
                liftMotorR.setPower(autonLiftPower);
                fourBarServo.setPosition(upPos);
                break;
            case LOW:
                liftMotorL.setTargetPosition(low);
                liftMotorL.setPower(autonLiftPower);
                liftMotorR.setTargetPosition(low);
                liftMotorR.setPower(autonLiftPower);
                fourBarServo.setPosition(upPos);
                break;
            case FLOOR:
                liftMotorL.setTargetPosition(floor);
                liftMotorL.setPower(autonLiftPower);
                liftMotorR.setTargetPosition(floor);
                liftMotorR.setPower(autonLiftPower);
                fourBarServo.setPosition(straightPos);
                break;
            case GRAB5:
                liftMotorL.setTargetPosition(stack5);
                liftMotorL.setPower(autonLiftPower);
                liftMotorR.setTargetPosition(stack5);
                liftMotorR.setPower(autonLiftPower);
                fourBarServo.setPosition(straightPos);
                break;
            case GRAB4:
                liftMotorL.setTargetPosition(stack4);
                liftMotorL.setPower(autonLiftPower);
                liftMotorR.setTargetPosition(stack4);
                liftMotorR.setPower(autonLiftPower);
                fourBarServo.setPosition(straightPos);
                break;
            case GRAB3:
                liftMotorL.setTargetPosition(stack3);
                liftMotorL.setPower(autonLiftPower);
                liftMotorR.setTargetPosition(stack3);
                liftMotorR.setPower(autonLiftPower);
                fourBarServo.setPosition(straightPos);
                break;
            case GRAB2:
                liftMotorL.setTargetPosition(stack2);
                liftMotorL.setPower(autonLiftPower);
                liftMotorR.setTargetPosition(stack2);
                liftMotorR.setPower(autonLiftPower);
                fourBarServo.setPosition(straightPos);
                break;
        }
    }

    public enum autonLiftPos {
        HIGH,
        MID,
        LOW,
        FLOOR,
        GRAB5,
        GRAB4,
        GRAB3,
        GRAB2,
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
        if(gamepad2.right_trigger > 0.1 && liftMotorR.getTargetPosition() < 1700 && liftMotorL.getTargetPosition() < 1700){
            liftMotorL.setTargetPosition((int) (liftMotorL.getCurrentPosition() + 90));
            liftMotorR.setTargetPosition((int) (liftMotorL.getCurrentPosition() + 90));
            liftMotorL.setPower(liftPower);
        } else if(gamepad2.left_trigger > 0.1 && liftMotorR.getTargetPosition() > 0 && liftMotorL.getTargetPosition() > 0){
            liftMotorL.setTargetPosition((int) (liftMotorL.getCurrentPosition() - 90));
            liftMotorR.setTargetPosition((int) (liftMotorL.getCurrentPosition() - 90));
            liftMotorL.setPower(liftPower);
        }
    }

    public void autonFourBar(boolean straight) {
        fourBarServo.setPosition(straight ? straightPos:upPos);
    }
    public void lowerAuton(double ticks){
        liftMotorL.setTargetPosition((int) liftMotorL.getTargetPosition() - (int) ticks);
        liftMotorR.setTargetPosition((int) liftMotorR.getTargetPosition() - (int) ticks);
        liftMotorL.setPower(autonLiftPower);
        liftMotorR.setPower(autonLiftPower);
    }
    public void autonFourBarPos(double pos){
        fourBarServo.setPosition(pos);
    }
}