package org.firstinspires.ftc.teamcode.commands

import org.firstinspires.ftc.teamcode.subsystems.vision.WebcamAprilTag
import org.firstinspires.ftc.teamcode.support.AlliancePresets
import org.openftc.apriltag.AprilTagDetection

class CypherAprilTagTracker(
    private val webcamAprilTag: WebcamAprilTag
) {
    // -1 means "unknown/not set yet"
    private var lastCypherId: Int = -1

    /** Call this repeatedly in init loop (or loop). Returns current remembered cypher ID. */
    fun update(): Int {
        val det: AprilTagDetection? = webcamAprilTag.tagOfinterest

        val newId = det?.id
        if (newId != null && isCypherId(newId)) {
            // Saw a valid cypher tag: update remembered id
            if (newId != lastCypherId) {
                lastCypherId = newId
            }
        }

        // Always push the remembered value to the global preset (if known)
        if (lastCypherId != -1) {
            AlliancePresets.setCurrentCypherId(lastCypherId)
        }

        return lastCypherId
    }

    /** Convenience: returns enum (or null if unknown). */
    fun getCypherEnum(): AlliancePresets.Cyphers? {
        return when (lastCypherId) {
            AlliancePresets.Cyphers.GPP.cypherTag -> AlliancePresets.Cyphers.GPP
            AlliancePresets.Cyphers.PGP.cypherTag -> AlliancePresets.Cyphers.PGP
            AlliancePresets.Cyphers.PPG.cypherTag -> AlliancePresets.Cyphers.PPG
            else -> null
        }
    }

    /** Raw ID (remembered). */
    fun getCypherId(): Int = lastCypherId

    /** Reset memory (optional). */
    fun reset() {
        lastCypherId = -1
    }

    private fun isCypherId(id: Int): Boolean {
        return id == AlliancePresets.Cyphers.GPP.cypherTag ||
                id == AlliancePresets.Cyphers.PGP.cypherTag ||
                id == AlliancePresets.Cyphers.PPG.cypherTag
    }
}
