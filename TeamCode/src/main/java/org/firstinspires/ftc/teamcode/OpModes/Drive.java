package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "DriveTesting")
public class Drive extends OpMode {
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
