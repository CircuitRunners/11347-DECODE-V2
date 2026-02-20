package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.Command
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import org.firstinspires.ftc.teamcode.subsystems.transfer.ColourZoneDetection
import org.firstinspires.ftc.teamcode.subsystems.transfer.Kickers

/**
 * FINAL executor:
 * - takes a precomputed plan
 * - for each zone in plan: kicker UP -> wait -> kicker DOWN -> wait
 * - reads ColourZoneDetection
 */
class ExecuteKickSequence(
    private val kickers: Kickers,
    private val planProvider: () -> List<ShotOrderPlanner.PlannedShot>,
    private val kickUpTimeS: Double = 0.18,
    private val resetTimeS: Double = 0.12
) : SequentialCommandGroup() {
    override fun initialize() {
        val plan = planProvider()

        val seq = ArrayList<Command>(plan.size * 4)
        for (shot in plan) {
            val zone: ColourZoneDetection.ZoneId = shot.zone
            seq += ZoneKickCommand(kickers, zone, true)
            seq += WaitCommand((kickUpTimeS * 1000.0).toLong())
            seq += ZoneKickCommand(kickers, zone, false)
            seq += WaitCommand((resetTimeS * 1000.0).toLong())
        }

        if (seq.isNotEmpty()) addCommands(*seq.toTypedArray())
        super.initialize()
    }
}
