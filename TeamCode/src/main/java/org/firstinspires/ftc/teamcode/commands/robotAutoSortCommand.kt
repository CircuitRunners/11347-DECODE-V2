package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.transfer.ColourZoneDetection

/**
 * RobotAutoSortCommand (planner-only version)
 * - reads ColourZoneDetection stable snapshot
 * - produces a plan (ordered zones) for max points based on selected cipher
 *
 * No kickers/shooter/turret. Intended for dashboard-driven testing.
 */
class RobotAutoSortCommand(
    private val colourZoneDetection: ColourZoneDetection,
    private val cipherProvider: () -> ShotOrderPlanner.Cipher
) : CommandBase() {

    private val planner = ShotOrderPlanner()

    var plan: List<ShotOrderPlanner.PlannedShot> = emptyList()
        private set

    override fun initialize() {
        colourZoneDetection.update()
        val snap = colourZoneDetection.getStableSnapshot()
        plan = planner.plan(cipherProvider(), snap)
    }

    override fun execute() {
        // Keep snapshots fresh; you can replan live if desired.
        colourZoneDetection.update()
    }

    override fun isFinished(): Boolean = false
}
