package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.subsystems.transfer.ColourZoneDetection

/**
 * ONE-SHOT sensor read + plan compute.
 * - Calls czd.update() exactly once at schedule-time.
 * - Reads a snapshot immediately after.
 * - Computes the plan once.
 * - Finishes immediately.
 *
 * This is intended to be scheduled inside a higher-level group that then runs ExecuteKickSequence.
 */
class RobotAutoSortCommand(
    private val czd: ColourZoneDetection,
    private val planner: ShotOrderPlanner,
    private val cipherProvider: () -> ShotOrderPlanner.Cipher,
    private val useRaw: Boolean = false,
    private val forceAllZones: Boolean = false,
    private val telemetry : Telemetry
) : CommandBase() {

    var plan: List<ShotOrderPlanner.PlannedShot> = emptyList()
        private set

    override fun initialize() {
        czd.update()

        val st = czd.rawSnapshot;

        telemetry.addData(
            "Z1 ST", "has=%s col=%s sum=%.4f a=%.4f cls=%s",
            st.z1.hasBall, st.z1.color, st.z1.sum, st.z1.a, st.z1.cls
        )
        telemetry.addData(
            "Z2 ST", "has=%s col=%s sum=%.4f a=%.4f cls=%s",
            st.z2.hasBall, st.z2.color, st.z2.sum, st.z2.a, st.z2.cls
        )
        telemetry.addData(
            "Z3 ST", "has=%s col=%s sum=%.4f a=%.4f cls=%s",
            st.z3.hasBall, st.z3.color, st.z3.sum, st.z3.a, st.z3.cls
        )

        val snap = if (useRaw) czd.rawSnapshot else czd.stableSnapshot
        plan = planner.plan(cipherProvider(), snap, forceAllZones)
    }

    override fun isFinished(): Boolean = true
}
