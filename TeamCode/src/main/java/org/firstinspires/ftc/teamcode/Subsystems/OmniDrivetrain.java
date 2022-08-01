package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class OmniDrivetrain {
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor strifeMotor;
    private HardwareMap hardwareMap;

    public OmniDrivetrain(OpMode opMode) {
        this.hardwareMap = opMode.hardwareMap;
    }

    public void init() {
        leftMotor = this.hardwareMap.get(DcMotor.class, "left");
        rightMotor = this.hardwareMap.get(DcMotor.class, "right");
        strifeMotor = this.hardwareMap.get(DcMotor.class, "strife");

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        strifeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void drive(double left, double right, double strife) {
        leftMotor.setPower(left);
        rightMotor.setPower(right);
        strifeMotor.setPower(strife);
    }
}
