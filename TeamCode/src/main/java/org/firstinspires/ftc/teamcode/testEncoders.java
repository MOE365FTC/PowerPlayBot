package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.rr.util.Encoder;
@TeleOp
public class testEncoders extends OpMode {
    DcMotorEx leftEncoder, rightEncoder, frontEncoder;
    @Override
    public void init() {
        leftEncoder = hardwareMap.get(DcMotorEx.class, "OLE11");
        rightEncoder = hardwareMap.get(DcMotorEx.class, "ORE13");
        frontEncoder = hardwareMap.get(DcMotorEx.class, "OCE12");

        rightEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
        frontEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        telemetry.addData("left", leftEncoder.getCurrentPosition());
        telemetry.addData("right", rightEncoder.getCurrentPosition());
        telemetry.addData("strafe", frontEncoder.getCurrentPosition());
    }
}
