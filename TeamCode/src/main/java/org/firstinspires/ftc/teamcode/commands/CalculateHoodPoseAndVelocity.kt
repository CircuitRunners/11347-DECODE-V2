package org.firstinspires.ftc.teamcode.commands

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.CommandBase
import com.pedropathing.geometry.Pose
import com.pedropathing.math.MathFunctions
import com.pedropathing.math.Vector
import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrivebase
import org.firstinspires.ftc.teamcode.subsystems.shooter.HoodSubsystem
import org.firstinspires.ftc.teamcode.subsystems.shooter.StaticShooter
import kotlin.math.atan
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.pow
import kotlin.math.sqrt
import kotlin.math.tan

@Config
class CalculateHoodPoseAndVelocity(
    private val drive: MecanumDrivebase,
    private val shooter : StaticShooter,
    private val hood : HoodSubsystem,
    private val goalPosRed: Pose,
    private val passThroughRadius: Double, // 5
    private val scoreHeight: Double, // 20
    private val scoreAngleRad: Double, // -30
    private val minHoodAngle: Double, // 0.0
    private val maxHoodAngle: Double,// 67
) : CommandBase() {
    private val timer = ElapsedTime()
    private val updatePeriodSec = 0.02
    var flywheelSpeed = 0.0
    var hoodAngle = 0.0
    var maxHoodTicks = 0.96 //0.9

    init {
        addRequirements( shooter, hood)
        timer.reset()
    }

    override fun execute() {
        if (timer.seconds() < updatePeriodSec) return
        timer.reset()

        val p = drive.pose
        val x = p.x
        val y = p.y

        calculateHoodAndFlywheel(x, y)
        val gearRatio = 1.0

        val wheelRPM = (flywheelSpeed * 60.0) / (Math.PI * (4.85 / 4.0))
        val motorRPM = wheelRPM * gearRatio

        val hoodPos = (maxHoodTicks - Range.scale(hoodAngle, minHoodAngle, maxHoodAngle, 0.0, 0.9)) // lower min value?

        shooter.targetRPM = motorRPM
        hood.aimScoring(hoodPos)
    }

    private fun calculateHoodAndFlywheel(robotX: Double, robotY: Double) {
        val dx = goalPosRed.x - robotX
        val dy = goalPosRed.y - robotY

        val distanceToGoal = hypot(dx, dy)
        val angleToGoal = atan2(dy, dx)
        val robotToGoalVector = Vector(distanceToGoal, angleToGoal)

        val g = 32.174 * 12.0
        val distX = robotToGoalVector.magnitude - passThroughRadius
        val heightY = scoreHeight
        val a = scoreAngleRad

        hoodAngle = MathFunctions.clamp(atan(2.0 * heightY / distX - tan(a)), minHoodAngle, maxHoodAngle)

        flywheelSpeed = sqrt(g * distX * distX / (2.0 * cos(hoodAngle).pow(2.0) * (distX * tan(hoodAngle) - heightY)))
    }
}
