package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystems.transfer.ColourZoneDetection
import org.firstinspires.ftc.teamcode.subsystems.transfer.Kickers

/**
 * One-shot: sets the selected zone's kicker UP or DOWN, then ends immediately.
 * Timing (kick up / reset) is handled by ExecuteKickSequence waits.
 */
class ZoneKickCommand(
    private val kickers: Kickers,
    private val zone: ColourZoneDetection.ZoneId,
    private val up: Boolean
) : CommandBase() {

    init {
        addRequirements(kickers)
    }

    override fun initialize() {
        when (zone) {
            ColourZoneDetection.ZoneId.Z1 -> if (up) kickers.kickZoneOne() else kickers.resetZoneOne()
            ColourZoneDetection.ZoneId.Z2 -> if (up) kickers.kickZoneTwo() else kickers.resetZoneTwo()
            ColourZoneDetection.ZoneId.Z3 -> if (up) kickers.kickZoneThree() else kickers.resetZoneThree()
        }
    }

    override fun isFinished(): Boolean = true
}
