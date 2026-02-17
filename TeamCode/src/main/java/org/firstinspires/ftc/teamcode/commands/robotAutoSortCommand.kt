// FILE: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/commands/RobotAutoSortCommand.kt
package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
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
    private val forceAllZones: Boolean = false
) : CommandBase() {

    var plan: List<ShotOrderPlanner.PlannedShot> = emptyList()
        private set

    override fun initialize() {
        czd.update()

        val snap = if (useRaw) czd.rawSnapshot else czd.stableSnapshot
        plan = planner.plan(cipherProvider(), snap, forceAllZones)
    }

    override fun isFinished(): Boolean = true
}
