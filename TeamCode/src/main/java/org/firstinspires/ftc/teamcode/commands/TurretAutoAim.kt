package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrivebase
import org.firstinspires.ftc.teamcode.subsystems.shooter.MotorTurretTracker
import kotlin.math.abs
import kotlin.math.min

class TurretAutoAim(
    private val drive: MecanumDrivebase,
    private val turret: MotorTurretTracker,
    private var turretTargetPose: Pose,
    private val isRed: Boolean
) : CommandBase() {

    init {
        addRequirements(turret)
    }

    override fun initialize() {
        turret.setEnabled(true)
    }

    override fun execute() {
        val pose = drive.pose
        val x = pose.x
        val y = pose.y
        val h = pose.heading
        val a = drive.angularVelocity

        val turretPose = Pose2D(
            DistanceUnit.INCH,
            x,
            y,
            AngleUnit.RADIANS,
            (if (abs(a) > 0.5 && (abs(a) < 5)) {
                h + (a * 0.18) //0.1 - 0.2
            } else if (abs(a) >= 5) {
                h + (a * 0.15)
            } else {
                h
            })
        )

        var gx = turretTargetPose.x
        var gy = turretTargetPose.y

        if (y < 60) {
            if (isRed) {
                gx += 2.0
                gy += 12.0
            } else {
                gx += 2.0
                gy += 10.0
            }
        } else if (y > 108) {
            if (isRed) {
                gx -= 0.0
                gy -= 4.0
            } else {
                gx -= 0.0
                gy -= 4.0
            }
        }

        turret.setTargetFieldPointInches(gx, gy)
        turret.updateAim(turretPose)
    }

    override fun end(interrupted: Boolean) {
        turret.setEnabled(false)
    }

    fun setTurretTargetPose(newTarget: Pose) {
        turretTargetPose = newTarget
    }
}
