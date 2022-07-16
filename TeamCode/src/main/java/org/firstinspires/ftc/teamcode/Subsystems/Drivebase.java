package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivebase {
    private DcMotor leftMaster;
    private DcMotor rightMaster;
    private HardwareMap hardwareMap;
    public Drivebase (OpMode opMode) {
        this.hardwareMap = opMode.hardwareMap;;
    }

    public void init() {
        leftMaster = hardwareMap.dcMotor.get("lm");
        rightMaster = hardwareMap.dcMotor.get("rm");

        leftMaster.setDirection(DcMotorSimple.Direction.REVERSE);

        leftMaster.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMaster.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setSpeed(double speed1, double speed2) {
        rightMaster.setPower(speed1);
        leftMaster.setPower(speed2);
    }
}
