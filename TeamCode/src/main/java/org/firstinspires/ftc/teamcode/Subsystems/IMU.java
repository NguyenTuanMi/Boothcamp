package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class IMU {
    private BNO055IMU imu;
    private Orientation angle;
    private HardwareMap hardwareMap;
    private AngularVelocity angularVelocity;
    private Acceleration accel;

    public IMU(OpMode opMode) {
        this.hardwareMap = opMode.hardwareMap;
    }

    public void init() {
        BNO055IMU.Parameters parameters = imu.getParameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu = hardwareMap.get(BNO055IMU.class, "IMU");
        imu.initialize(parameters);
    }

    public void updateAngularVelocity() {
        angularVelocity = imu.getAngularVelocity();
    }

    public void updateAngleOrientation() {
        angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);
    }

    public void updateAccel() {
        accel = imu.getAcceleration();
    }

    public AngularVelocity getAngularVelocity() {
        return angularVelocity;
    }

    public Orientation getAngleOrientation() {
        return angle;
    }

    //public Acceleration getAccel() {
    //    return accel;
    //}

    public float getYaw() {
        return getAngleOrientation().thirdAngle;
    }

    public float getRoll() {
        return getAngleOrientation().firstAngle;
    }

    public float getPitch() {
        return getAngleOrientation().secondAngle;
    }

    public float getRotaRateX() {
        return getAngularVelocity().xRotationRate;
    }

    public float getRotaRateY() {
        return getAngularVelocity().yRotationRate;
    }

    public float getRotaRateZ() {
        return getAngularVelocity().zRotationRate;
    }

    public double getAccelX() {
        return accel.xAccel;
    }

    public double getAccelY() {
        return accel.yAccel;
    }

    public double getAccelZ() {
        return accel.zAccel;
    }


}
