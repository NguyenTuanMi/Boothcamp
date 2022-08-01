package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Drivetrain {
    private DcMotorEx leftMaster;
    private DcMotorEx rightMaster;
    private DcMotorEx leftFollow;
    private DcMotorEx rightFollow;
    private HardwareMap hardwareMap;

    public Drivetrain(OpMode opMode) {
        this.hardwareMap = opMode.hardwareMap;;
    }

    public void init() {
        leftMaster = hardwareMap.get(DcMotorEx.class, "lm");
        rightMaster = hardwareMap.get(DcMotorEx.class, "rm");
        leftFollow = hardwareMap.get(DcMotorEx.class, "lf");
        rightFollow = hardwareMap.get(DcMotorEx.class, "rf");


        leftMaster.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFollow.setDirection(DcMotorSimple.Direction.REVERSE);

        leftMaster.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMaster.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFollow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFollow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setSpeed(double speed1, double speed2) {
        rightMaster.setPower(speed1);
        leftMaster.setPower(speed2);
        rightFollow.setPower(speed1);
        leftFollow.setPower(speed2);
    }

    public double getLeftVelocity() {
        return leftMaster.getVelocity(AngleUnit.RADIANS);
    }

    public double getRightVelocity() {
        return rightMaster.getVelocity(AngleUnit.RADIANS);
    }
}
