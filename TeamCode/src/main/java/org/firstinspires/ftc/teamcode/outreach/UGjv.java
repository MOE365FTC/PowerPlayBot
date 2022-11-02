package org.firstinspires.ftc.teamcode.outreach;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class UGjv extends OpMode {
    UGBot bot = new UGBot();
    @Override
    public void init() {
        bot.init();
    }

    @Override
    public void loop() {
        bot.loop();
    }
}
