package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class OmniDrivetrain {
    private DcMotorEx leftMotor;
    private DcMotorEx rightMotor;
    private DcMotorEx strifeMotor;
    private HardwareMap hardwareMap;

    public OmniDrivetrain(OpMode opMode) {
        this.hardwareMap = opMode.hardwareMap;
    }

    public void init() {
        leftMotor = this.hardwareMap.get(DcMotorEx.class, "left");
        rightMotor = this.hardwareMap.get(DcMotorEx.class, "right");
        strifeMotor = this.hardwareMap.get(DcMotorEx.class, "strife");

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

    public double getLeftVelocity() { return leftMotor.getVelocity(AngleUnit.RADIANS);}
    public double getRightVelocity() { return rightMotor.getVelocity(AngleUnit.RADIANS);}
    public double getStrifeVelocity() { return strifeMotor.getVelocity(AngleUnit.RADIANS); }

}
