package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.Arm;
//import org.firstinspires.ftc.teamcode.Subsystems.Drivebase;

public class Robot {
    //private Drivebase drivebase;
    private Arm arm;
    private Gamepad gamepad;
    public Robot(OpMode opMode) {
        //drivebase = new Drivebase(opMode);
        arm = new Arm(opMode);
        gamepad = opMode.gamepad1;
    }

    public void init() {
        //drivebase.init();
        arm.init();
    }
    public void loop() {
        //drivebase.setSpeed(gamepad.right_stick_y, gamepad.left_stick_y);
        double speed1 = 0;
        double speed2 = 0;
        if (gamepad.left_bumper) {
            if (gamepad.a) {
                speed1 = -0.3;
            }
            if (gamepad.b) {
                speed2 = -0.3;
            }
        }

        if (gamepad.a) {
            speed1 = 0.3;
        }

        if (gamepad.b) {
            speed2 = 0.3;
        }
    }
}
