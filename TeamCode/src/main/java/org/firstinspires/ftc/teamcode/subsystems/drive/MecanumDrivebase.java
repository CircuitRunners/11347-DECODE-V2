package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


public class MecanumDrivebase extends SubsystemBase {
    public DcMotorEx frontLeftMotor;
    public DcMotorEx frontRightMotor;
    public DcMotorEx backLeftMotor;
    public DcMotorEx backRightMotor;
    private Follower follower;
    private boolean isRed = false;
    private boolean isAuto = false;

    public MecanumDrivebase(HardwareMap hardwareMap, boolean isRed, boolean isAuto) {
        this(hardwareMap, isRed, isAuto, Constants.createFollower(hardwareMap));
    }

    public MecanumDrivebase(HardwareMap hardwareMap, boolean isRed, boolean isAuto, Follower follower) {
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "fl");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "fr");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "bl");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "br");

        this.follower = follower;
        this.isRed = isRed;
        this.isAuto = isAuto;

        if (!isAuto) {
            setStartingPose();
        }

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotorEx[] motors = new DcMotorEx[]{
                frontLeftMotor,frontRightMotor, backLeftMotor, backRightMotor
        };


        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    private void setPowers(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
        double maxSpeed = 1.0;
        maxSpeed = Math.max(maxSpeed, Math.abs(frontLeftPower));
        maxSpeed = Math.max(maxSpeed, Math.abs(frontRightPower));
        maxSpeed = Math.max(maxSpeed, Math.abs(backLeftPower));
        maxSpeed = Math.max(maxSpeed, Math.abs(backRightPower));
        frontLeftPower /= maxSpeed;
        frontRightPower /= maxSpeed;
        backLeftPower /= maxSpeed;
        backRightPower /= maxSpeed;
        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
    }
    public void drive(double forward, double right, double rotate) {
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backLeftPower = forward - right + rotate;
        double backRightPower = forward + right - rotate;

        setPowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    public void driveFieldRelative(double forward, double right, double rotate, boolean reverseHeading, Follower follower) {
        follower.update();

        Pose pos = follower.getPose();

        double robotAngle = reverseHeading ? pos.getHeading() : pos.getHeading() - Math.toRadians(180);
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(forward, right);
        theta = AngleUnit.normalizeRadians(theta - robotAngle);

        double newForward   = r * Math.sin(theta);
        double newRight     = r * Math.cos(theta);

        drive(newForward, newRight, rotate);
    }

    public void driveFieldRelative(double forward, double right, double rotate, boolean reverseHeading) {
        driveFieldRelative(forward, right, rotate, reverseHeading, follower);
    }

    public Pose getPose() {
        return follower.getPose();
    }

    public Vector getVelocity() {
        return follower.getVelocity();
    }

    public void setPose(Pose pose) {
        follower.setPose(pose);
    }

    public void setStartingPose() {
        follower.setPose(new Pose(72, 72, isRed ? Math.toRadians(0.0) : Math.toRadians(180.0)));
    }
}
