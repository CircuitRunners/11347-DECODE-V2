package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class IntakeSubsystem extends SubsystemBase {
    private DcMotorEx intake;

    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Intake Init Done");
    }

    public void intake(double power) {
        intake.setPower(Range.clip(power, -0.9, 0.9));
    }

    public void stop() {
        intake(0);
    }
}
