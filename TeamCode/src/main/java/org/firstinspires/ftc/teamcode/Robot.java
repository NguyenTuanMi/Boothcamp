package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.IMU;
import org.firstinspires.ftc.teamcode.Subsystems.OmniDrivetrain;

public class Robot {
    private Drivetrain drivetrain;
    private OmniDrivetrain omni;
    private IMU imu;
    private Telemetry telemetry;
    private Gamepad gamepad;
    private double axial = 0;
    private double lateral = 0;
    private double yaw = 0;
    private double leftPowerMode = 0;
    private double rightPowerMode = 0;
    private double strifePowerMode = 0;

    public Robot(OpMode opMode) {
        drivetrain = new Drivetrain(opMode);
        omni = new OmniDrivetrain(opMode);
        imu = new IMU(opMode);
        this.telemetry = opMode.telemetry;
        this.gamepad = opMode.gamepad1;
    }

    public void init() {
        drivetrain.init();
        omni.init();
        imu.init();
    }

    public void loop() {
        this.telemetry.addData("IMU yaw value", imu.getYaw());
        this.telemetry.addData("IMU roll value", imu.getRoll());
        this.telemetry.addData("IMU pitch value", imu.getPitch());

        this.telemetry.addData("IMU accel x", imu.getAccelX());
        this.telemetry.addData("IMU accel y", imu.getAccelY());
        this.telemetry.addData("IMU accel z", imu.getAccelZ());

        this.telemetry.addData("IMU rotation rate x", imu.getRotaRateX());
        this.telemetry.addData("IMU rotation rate y", imu.getRotaRateY());
        this.telemetry.addData("IMU rotation rate z", imu.getRotaRateZ());

        this.telemetry.update();
    }

    public void OmniDriveController() {
        axial = this.gamepad.left_stick_y;
        lateral = this.gamepad.right_stick_y;
        yaw = this.gamepad.left_stick_x;

        leftPowerMode = axial + yaw;
        rightPowerMode = axial - yaw;
        strifePowerMode = lateral;

        omni.drive(leftPowerMode, rightPowerMode, lateral);
    }

    public void TankDriveController() {
        drivetrain.setSpeed(this.gamepad.right_stick_y, this.gamepad.left_stick_y);
    }
}
