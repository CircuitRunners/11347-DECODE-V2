package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
@Config
public class tiltClimb extends SubsystemBase {
    private final CRServo tilt;
    private final ElapsedTime climbTimer;

    public static double power = 0;
    private boolean automatic = false;
    private boolean runningSequence = false;

    public tiltClimb(HardwareMap hardwareMap) {
        tilt = hardwareMap.get(CRServo.class, "tilt");
        climbTimer = new ElapsedTime();
    }

    @Override
    public void periodic() {
        if (runningSequence) {
            double t = climbTimer.seconds();

            if (t < 3.0) {
                tilt.setPower(1.0);
            } else if (t < 6.9 && t > 3.05) {
                tilt.setPower(1.0);
            } else {
                tilt.setPower(0.0);
            }

            if (t > 7.0) {
                automatic = false;
                runningSequence = false;
            }
        } else if (!automatic) {
            tilt.setPower(power);
        }
    }

    public void tilt(double power) {
        tilt.setPower(power);
    }

    public void setAutomatic(boolean bool) {
        automatic = bool;
    }

    public void startTiltSequence() {
        automatic = true;
        runningSequence = true;
        climbTimer.reset();
    }

    public boolean isRunningSequence() {
        return runningSequence;
    }
}
