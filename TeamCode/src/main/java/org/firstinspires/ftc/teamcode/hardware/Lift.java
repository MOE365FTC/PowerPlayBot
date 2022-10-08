package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {
    HardwareMap hardwareMap;
    Gamepad gamepad1;

    DcMotor liftMotor;

    int high = 600, mid = 400, low = 200, floor = 100;
    double liftPower;
    public Lift(HardwareMap hardwareMap, Gamepad gamepad1){
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad1;

        liftMotor = hardwareMap.get(DcMotor.class, "SLM");
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void actuate(){
        if(gamepad1.y){
            liftMotor.setTargetPosition(high);
            liftMotor.setPower(liftPower);
        } else if (gamepad1.x){
            liftMotor.setTargetPosition(mid);
            liftMotor.setPower(liftPower);
        } else if (gamepad1.b){
            liftMotor.setTargetPosition(low);
            liftMotor.setPower(liftPower);
        } else if (gamepad1.a){
            liftMotor.setTargetPosition(floor);
            liftMotor.setPower(liftPower);
        }
    }
}
