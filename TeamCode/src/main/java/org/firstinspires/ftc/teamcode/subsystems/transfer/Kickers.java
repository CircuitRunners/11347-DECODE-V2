package org.firstinspires.ftc.teamcode.subsystems.transfer;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.support.RunAction;

@Config
public class Kickers extends SubsystemBase {
    public enum KickerState {
        UP(0.21, 0.23, 0.21),
        DOWN(0.54, 0.51, 0.52);

        public final double kickerOne, kickerTwo, kickerThree;
        KickerState(double kickerOne, double kickerTwo, double kickerThree) {
            this.kickerOne = kickerOne;
            this.kickerTwo = kickerTwo;
            this.kickerThree = kickerThree;
        }

        public double getKickerOnePosition() {
            return kickerOne;
        }

        public double getKickerTwoPosition() {
            return kickerTwo;
        }

        public double getKickerThreePosition() {
            return kickerThree;
        }
    }

    private Servo kickerServo1, kickerServo2, kickerServo3;
    public RunAction kickZoneOne, kickZoneTwo, kickZoneThree,
                    resetZoneOne, resetZoneTwo, resetZoneThree;

    public Kickers(HardwareMap hardwareMap) {
        kickerServo1 = hardwareMap.get(Servo.class, "ks1");
        kickerServo2 = hardwareMap.get(Servo.class, "ks2");
        kickerServo3 = hardwareMap.get(Servo.class, "ks3");

        setAllKickers(KickerState.DOWN);

        kickZoneOne = new RunAction(this::kickZoneOne);
        kickZoneTwo = new RunAction(this::kickZoneTwo);
        kickZoneThree = new RunAction(this::kickZoneThree);
        resetZoneOne = new RunAction(this::resetZoneOne);
        resetZoneTwo = new RunAction(this::resetZoneTwo);
        resetZoneThree = new RunAction(this::resetZoneThree);
    }

    public void setAllKickers(KickerState state) {
        kickerServo1.setPosition(state.getKickerOnePosition());
        kickerServo2.setPosition(state.getKickerTwoPosition());
        kickerServo3.setPosition(state.getKickerThreePosition());
    }

    /// ========================== Kicker Up States ==========================
    public void kickZoneOne() {
        kickerServo1.setPosition(KickerState.UP.getKickerOnePosition());
    }

    public void kickZoneTwo() {
        kickerServo2.setPosition(KickerState.UP.getKickerTwoPosition());
    }

    public void kickZoneThree() {
        kickerServo3.setPosition(KickerState.UP.getKickerThreePosition());
    }

    /// ======================== Kicker Down States ========================
    public void resetZoneOne() {
        kickerServo1.setPosition(KickerState.DOWN.getKickerOnePosition());
    }

    public void resetZoneTwo() {
        kickerServo2.setPosition(KickerState.DOWN.getKickerTwoPosition());
    }

    public void resetZoneThree() {
        kickerServo3.setPosition(KickerState.DOWN.getKickerThreePosition());
    }

    public void setZoneOnePos(double p) {
        kickerServo1.setPosition(p);
    }
    public void setZoneTwoPos(double p) {
        kickerServo2.setPosition(p);
    }
    public void setZoneThreePos(double p) {
        kickerServo3.setPosition(p);
    }
}
