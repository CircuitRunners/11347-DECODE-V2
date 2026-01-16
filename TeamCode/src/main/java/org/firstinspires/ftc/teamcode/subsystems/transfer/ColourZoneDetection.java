package org.firstinspires.ftc.teamcode.subsystems.transfer;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.support.SRSHub;

@Config
public class ColourZoneDetection extends SubsystemBase {

    /// ============================== Public types ==============================
    public enum BallColor { PURPLE, GREEN, NONE }

    public enum ZoneId { Z1, Z2, Z3 }

    public static class ZoneState {
        public final ZoneId zone;
        public final boolean hasBall;
        public final BallColor color;

        // raw debug (primary chosen sensor)
        public final int r, g, b;
        public final short prox;

        public ZoneState(ZoneId zone, boolean hasBall, BallColor color,
                         int r, int g, int b, short prox) {
            this.zone = zone;
            this.hasBall = hasBall;
            this.color = color;
            this.r = r;
            this.g = g;
            this.b = b;
            this.prox = prox;
        }
    }

    public static class Snapshot {
        public final ZoneState z1, z2, z3;
        public Snapshot(ZoneState z1, ZoneState z2, ZoneState z3) {
            this.z1 = z1; this.z2 = z2; this.z3 = z3;
        }
    }
    
}
