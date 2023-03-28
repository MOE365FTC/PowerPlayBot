package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Lift {
    HardwareMap hardwareMap;
    Gamepad gamepad2;

    public DcMotor liftMotor;
    public Servo forkServo;
    int high = 1440, mid = 860, low = 350, floor = 15, stack5 = 280, stack4 = 250, stack3 = 60, stack2 = 30; //lift extension
    int maxLiftPos = 1700; //FIND THIS, TEMP VALUE FOR RIGHT NOW
    int manualIncrement = 90;
    double liftPower = 0.7;
    double autonLiftPower = 0.55;
    public Lift(HardwareMap hardwareMap, Gamepad gamepad2){
        this.hardwareMap = hardwareMap;
        this.gamepad2 = gamepad2;

        liftMotor = hardwareMap.get(DcMotor.class, "RLM01");
        forkServo = hardwareMap.get(Servo.class, "forkServo");

        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void actuate(){
        if(gamepad2.y){
            liftMotor.setTargetPosition(high);
            liftMotor.setPower(liftPower);
        } else if (gamepad2.x){
            liftMotor.setTargetPosition(mid);
            liftMotor.setPower(liftPower);
        } else if (gamepad2.b){
            liftMotor.setTargetPosition(low);
            liftMotor.setPower(liftPower);
        } else if (gamepad2.a) {
            liftMotor.setTargetPosition(floor);
            liftMotor.setPower(liftPower);
        } else if (gamepad2.dpad_up){
            liftMotor.setTargetPosition(stack5);
            liftMotor.setPower(liftPower);
        }else if (gamepad2.dpad_right){
            liftMotor.setTargetPosition(stack4);
            liftMotor.setPower(liftPower);
        } else if (gamepad2.dpad_left){
            liftMotor.setTargetPosition(stack3);
            liftMotor.setPower(liftPower);
        } else if (gamepad2.dpad_down){
            liftMotor.setTargetPosition(stack2);
            liftMotor.setPower(liftPower);
        }
    }

    public void autonActuate(autonLiftPos pos) {
        switch (pos) {
            case HIGH:
                liftMotor.setTargetPosition(high);
                liftMotor.setPower(autonLiftPower);
                break;
            case MID:
                liftMotor.setTargetPosition(mid);
                liftMotor.setPower(autonLiftPower);
                break;
            case LOW:
                liftMotor.setTargetPosition(low);
                liftMotor.setPower(autonLiftPower);
                break;
            case FLOOR:
                liftMotor.setTargetPosition(floor);
                liftMotor.setPower(autonLiftPower);
                break;
            case GRAB5:
                liftMotor.setTargetPosition(stack5);
                liftMotor.setPower(autonLiftPower);
                break;
            case GRAB4:
                liftMotor.setTargetPosition(stack4);
                liftMotor.setPower(autonLiftPower);
                break;
            case GRAB3:
                liftMotor.setTargetPosition(stack3);
                liftMotor.setPower(autonLiftPower);
                break;
            case GRAB2:
                liftMotor.setTargetPosition(stack2);
                liftMotor.setPower(autonLiftPower);
                break;
        }
    }

    public void forkActuate(boolean up) {
        forkServo.setPosition(up ? 0.0 : 1.0);
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

    public int getLiftTicks() {
        return liftMotor.getCurrentPosition();
    }


    public void manualLift(){
        if(gamepad2.right_trigger > 0.1 && liftMotor.getTargetPosition() < maxLiftPos){
            liftMotor.setTargetPosition((liftMotor.getCurrentPosition() + manualIncrement));
            liftMotor.setPower(liftPower);
        } else if(gamepad2.left_trigger > 0.1 && liftMotor.getTargetPosition() > 0){
            liftMotor.setTargetPosition((liftMotor.getCurrentPosition() - manualIncrement));
            liftMotor.setPower(liftPower);
        }
    }

    public void lowerAuton(double ticks){
        liftMotor.setTargetPosition(liftMotor.getTargetPosition() - (int) ticks);
        liftMotor.setPower(autonLiftPower);
    }
}