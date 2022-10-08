package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    Servo clawServo;
    Gamepad gamepad1;
    HardwareMap hardwareMap;

    double open = 1.0, closed = 0.0;
    //teleop
    public Claw(HardwareMap hardwareMap, Gamepad gamepad1){
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;

        clawServo = hardwareMap.get(Servo.class, "CGM");
    }

    public void grab(){
        clawServo.setPosition(closed);
    }

    public void release(){
        clawServo.setPosition(open);
    }

}
