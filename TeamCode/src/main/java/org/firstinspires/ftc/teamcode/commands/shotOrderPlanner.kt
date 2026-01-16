package org.firstinspires.ftc.teamcode.commands

import org.firstinspires.ftc.teamcode.subsystems.transfer.ColourZoneDetection

/**
 * ShotOrderPlanner
 *
 * Input:
 *  - cipher (PPG / PGP / GPP)
 *  - ColourZoneDetection.Snapshot (stable zone states)
 *
 * Output:
 *  - ordered list of zones to shoot (only includes zones that actually match desired colors)
 *
 * Notes:
 *  - This planner is "safe": it won't tell you to shoot a zone unless it contains the desired color.
 *  - If you want fallback behavior (shoot remaining balls anyway), enable it below.
 */
class ShotOrderPlanner {

    enum class Cipher { PPG, PGP, GPP }

    data class PlannedShot(
        val zone: ColourZoneDetection.ZoneId,
        val color: ColourZoneDetection.BallColor
    )

    fun plan(cipher: Cipher, snap: ColourZoneDetection.Snapshot): List<PlannedShot> {
        val desired = when (cipher) {
            Cipher.PPG -> listOf(ColourZoneDetection.BallColor.PURPLE, ColourZoneDetection.BallColor.PURPLE, ColourZoneDetection.BallColor.GREEN)
            Cipher.PGP -> listOf(ColourZoneDetection.BallColor.PURPLE, ColourZoneDetection.BallColor.GREEN, ColourZoneDetection.BallColor.PURPLE)
            Cipher.GPP -> listOf(ColourZoneDetection.BallColor.GREEN, ColourZoneDetection.BallColor.PURPLE, ColourZoneDetection.BallColor.PURPLE)
        }

        val zones = mutableListOf(
            ColourZoneDetection.ZoneId.Z1 to snap.z1,
            ColourZoneDetection.ZoneId.Z2 to snap.z2,
            ColourZoneDetection.ZoneId.Z3 to snap.z3
        )

        val plan = mutableListOf<PlannedShot>()

        for (want in desired) {
            val idx = zones.indexOfFirst { (_, st) ->
                st.hasBall && st.color == want
            }
            if (idx >= 0) {
                val (zoneId, _) = zones.removeAt(idx)
                plan += PlannedShot(zoneId, want)
            }
        }

        // Optional fallback: if cipher can't be fully satisfied, shoot whatever remains (to clear balls)
        // Uncomment if your game rules reward clearing vs holding.
        /*
        zones.filter { it.second.hasBall && it.second.color != ColourZoneDetection.BallColor.NONE }
            .forEach { (zoneId, st) -> plan += PlannedShot(zoneId, st.color) }
        */

        return plan
    }
}
