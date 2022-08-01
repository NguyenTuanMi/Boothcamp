package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "IMU data checking")
public class DataDisplayer extends OpMode {
    private Robot robot;
    @Override
    public void init() {
        robot = new Robot(this);
        robot.init();
    }

    @Override
    public void loop() {
        robot.loop();
    }
}
