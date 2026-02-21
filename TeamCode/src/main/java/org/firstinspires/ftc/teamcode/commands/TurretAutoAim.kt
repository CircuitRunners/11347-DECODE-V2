package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import com.pedropathing.geometry.Pose
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrivebase
import org.firstinspires.ftc.teamcode.subsystems.shooter.ServoTurretTracker
import org.firstinspires.ftc.teamcode.support.OdoAbsoluteHeadingTracking

class TurretAutoAim(
    private val drive : MecanumDrivebase,
    private val turret : ServoTurretTracker,
    private val odoHeading : OdoAbsoluteHeadingTracking,
    private val turretTargetPose : Pose
) : CommandBase() {
    init {
        addRequirements(turret)
    }

    override fun initialize() {
        turret.isEnabled = true
    }

    override fun execute() {
        val pose = drive.pose
        val x = pose.x
        val y = pose.y
        val h = odoHeading.headingRad

        val turretPose = Pose2D(
            DistanceUnit.INCH,
            x,
            y,
            AngleUnit.RADIANS,
            h
        )

        val targetX : Double = turretTargetPose.x
        val targetY : Double = turretTargetPose.y

        var gx : Double = targetX
        val gy : Double = targetY

        if (y < 60) gx -= 4

        turret.setTargetFieldPointInches(gx, gy)
        turret.update(turretPose)
    }

    override fun end(interrupted: Boolean) {
        turret.isEnabled = false
    }
}
