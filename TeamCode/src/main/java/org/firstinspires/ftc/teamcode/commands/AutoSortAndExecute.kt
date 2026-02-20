package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.subsystems.transfer.ColourZoneDetection
import org.firstinspires.ftc.teamcode.subsystems.transfer.Kickers

/**
 * Button schedules THIS:
 * 1) update CZD once + plan once
 * 2) execute kickers from that cached plan
 */
class AutoSortAndExecute(
    private val czd: ColourZoneDetection,
    private val kickers: Kickers,
    private val planner: ShotOrderPlanner,
    private val cipherProvider: () -> ShotOrderPlanner.Cipher,
    private val kickUpTimeS: Double = 0.18,
    private val resetTimeS: Double = 0.12,
    private val useRaw: Boolean = false,
    private val forceAllZones: Boolean = false,
    private val telemetry : Telemetry
) : SequentialCommandGroup() {

    private var cachedPlan: List<ShotOrderPlanner.PlannedShot> = emptyList()

    init {
        val autoSort = RobotAutoSortCommand(
            czd = czd,
            planner = planner,
            cipherProvider = cipherProvider,
            useRaw = useRaw,
            forceAllZones = forceAllZones,
            telemetry = telemetry
        )

        addCommands(
            autoSort,
            InstantCommand({ cachedPlan = autoSort.plan }),
            ExecuteKickSequence(
                kickers = kickers,
                planProvider = { cachedPlan },
                kickUpTimeS = kickUpTimeS,
                resetTimeS = resetTimeS
            )
        )
    }
}
