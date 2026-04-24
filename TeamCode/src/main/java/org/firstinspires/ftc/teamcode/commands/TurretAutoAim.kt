package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import com.pedropathing.geometry.Pose
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrivebase
import org.firstinspires.ftc.teamcode.subsystems.shooter.MotorTurretTracker

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

        val turretPose = Pose2D(
            DistanceUnit.INCH,
            x,
            y,
            AngleUnit.RADIANS,
            h
        )

        var gx = turretTargetPose.x
        val gy = turretTargetPose.y

        if (y < 60) {
//            if (isRed) {
//                gx -= 2.0
//            } else {
//                gx -= 2.0
//            }
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
