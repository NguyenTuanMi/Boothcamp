package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Arm controller")
public class ArmController extends OpMode {
    private Arm arm;
    @Override
    public void init() {
        arm = new Arm(this);
        arm.getController().pwmEnable();
        arm.setGatePosition(0);
    }

    @Override
    public void loop() {
        if (this.gamepad1.circle) {
            arm.setGatePosition(0.5);
        }
    }

    @Override
    public void stop() {
        arm.setGatePosition(0.5);
    }
}
