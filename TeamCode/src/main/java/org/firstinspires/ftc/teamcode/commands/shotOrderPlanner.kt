package org.firstinspires.ftc.teamcode.commands

import org.firstinspires.ftc.teamcode.subsystems.transfer.ColourZoneDetection

class ShotOrderPlanner {

    enum class Cipher { PPG, PGP, GPP }

    data class PlannedShot(
        val zone: ColourZoneDetection.ZoneId,
        val color: ColourZoneDetection.BallColor
    )

    /**
     * @param forceAllZones If true, always return 3 shots (Z1,Z2,Z3) even if sensors say empty/disconnected.
     */
    fun plan(
        cipher: Cipher,
        snap: ColourZoneDetection.Snapshot,
        forceAllZones: Boolean = false
    ): List<PlannedShot> {

        // If we want guaranteed 3 shots, ignore sensors and just shoot all zones in a fixed order.
        // Color is informational only.
        if (forceAllZones) {
            return listOf(
                PlannedShot(ColourZoneDetection.ZoneId.Z1, snap.z1.color),
                PlannedShot(ColourZoneDetection.ZoneId.Z2, snap.z2.color),
                PlannedShot(ColourZoneDetection.ZoneId.Z3, snap.z3.color)
            )
        }

        val desired: List<ColourZoneDetection.BallColor> = when (cipher) {
            Cipher.PPG -> listOf(
                ColourZoneDetection.BallColor.PURPLE,
                ColourZoneDetection.BallColor.PURPLE,
                ColourZoneDetection.BallColor.GREEN
            )
            Cipher.PGP -> listOf(
                ColourZoneDetection.BallColor.PURPLE,
                ColourZoneDetection.BallColor.GREEN,
                ColourZoneDetection.BallColor.PURPLE
            )
            Cipher.GPP -> listOf(
                ColourZoneDetection.BallColor.GREEN,
                ColourZoneDetection.BallColor.PURPLE,
                ColourZoneDetection.BallColor.PURPLE
            )
        }

        // Only shoot zones that actually have a ball
        val present = listOf(
            ColourZoneDetection.ZoneId.Z1 to snap.z1,
            ColourZoneDetection.ZoneId.Z2 to snap.z2,
            ColourZoneDetection.ZoneId.Z3 to snap.z3
        ).filter { (_, st) -> st.hasBall && st.color != ColourZoneDetection.BallColor.NONE }

        if (present.isEmpty()) return emptyList()
        if (present.size == 1) {
            val (z, st) = present[0]
            return listOf(PlannedShot(z, st.color))
        }

        // Evaluate all permutations
        val perms = permutations(present)
        var best: List<Pair<ColourZoneDetection.ZoneId, ColourZoneDetection.ZoneState>> = perms[0]
        var bestScore = score(best, desired)

        for (i in 1 until perms.size) {
            val cand = perms[i]
            val s = score(cand, desired)
            if (s > bestScore) {
                bestScore = s
                best = cand
            }
        }

        return best.map { (z, st) -> PlannedShot(z, st.color) }
    }

    private fun score(
        order: List<Pair<ColourZoneDetection.ZoneId, ColourZoneDetection.ZoneState>>,
        desired: List<ColourZoneDetection.BallColor>
    ): Int {
        var s = 0
        val weights = intArrayOf(3, 2, 1)
        val n = minOf(order.size, desired.size)
        for (i in 0 until n) {
            if (order[i].second.color == desired[i]) s += weights[i]
        }
        return s
    }

    private fun <T> permutations(list: List<T>): List<List<T>> {
        if (list.size <= 1) return listOf(list)
        val result = mutableListOf<List<T>>()
        for (i in list.indices) {
            val head = list[i]
            val rest = list.toMutableList().also { it.removeAt(i) }
            for (p in permutations(rest)) {
                result.add(listOf(head) + p)
            }
        }
        return result
    }
}
