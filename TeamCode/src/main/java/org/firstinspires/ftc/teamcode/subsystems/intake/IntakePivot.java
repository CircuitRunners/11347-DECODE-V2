package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class IntakePivot extends SubsystemBase {
    private final Servo intakePivot;
    public static double pivotTuning = 0.25;
    public static double PIVOT_MAX = 0.35;
    public static double ZERO_POSE = 0.25;
    public static double PIVOT_INTAKE = 0.29;

    public IntakePivot(HardwareMap hardwareMap) {
        intakePivot = hardwareMap.get(Servo.class, "intakeServo");

        intakePivot.setPosition(ZERO_POSE);
    }

    public void setPivotMax() {
        setPivot(PIVOT_MAX);
    }

    public void setPivotZero() {
        setPivot(ZERO_POSE);
    }

    public void setPivotIntaking() {
        setPivot(PIVOT_INTAKE);
    }

    public void setPivot(double pose) {
        intakePivot.setPosition(pose);
    }
}
