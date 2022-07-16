package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

public class Arm {
    private DcMotor motor1;
    private DcMotor motor2;
    private Servo servo;
    private HardwareMap hardwareMap;
    public Arm(OpMode opMode) {
        hardwareMap = opMode.hardwareMap;
    }

    public void init() {
        servo = hardwareMap.servo.get("gate");

        servo.setDirection(Servo.Direction.FORWARD);

        motor1 = hardwareMap.dcMotor.get("motor1");
        motor2 = hardwareMap.dcMotor.get("motor2");

        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2.setDirection(DcMotorSimple.Direction.FORWARD);

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setGatePosition(double position) {
        servo.setPosition(position);
    }

    public void setSpeed(double speed1, double speed2) {
        motor1.setPower(speed1);
        motor2.setPower(speed2);
    }

    public ServoController getController() {
        return servo.getController();
    }


}
