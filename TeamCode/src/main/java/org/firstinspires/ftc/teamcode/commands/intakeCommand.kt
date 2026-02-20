package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.transfer.ColourZoneDetection
import org.firstinspires.ftc.teamcode.support.GobildaRGBIndicatorHelper

class IntakeCommand(
    private val inSubsystem : IntakeSubsystem,
    private val driver : GamepadEx,
    private val czd : ColourZoneDetection,
    private val rgb : GobildaRGBIndicatorHelper
) : CommandBase() {
    private var wasIntaking = false

    init {
        addRequirements(inSubsystem, czd, rgb)
    }

    override fun execute() {
        val leftTrigger = driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)
        val rightTrigger = driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)
        val isIntaking = (leftTrigger > 0.1 || rightTrigger > 0.1)

        if (isIntaking) {
            inSubsystem.intake(leftTrigger - rightTrigger)
        } else {
            inSubsystem.stop()

            if (wasIntaking) {
                czd.update()
                val snapshot = czd.rawSnapshot
                val ballCount =
                    (if (snapshot.z1.hasBall) 1 else 0) +
                    (if (snapshot.z2.hasBall) 1 else 0) +
                    (if (snapshot.z3.hasBall) 1 else 0)

                when (ballCount) {
                    1 -> rgb.setColour(GobildaRGBIndicatorHelper.Colour.YELLOW)
                    2 -> rgb.setColour(GobildaRGBIndicatorHelper.Colour.GREEN)
                    3 -> rgb.setColour(GobildaRGBIndicatorHelper.Colour.VIOLET)
                    else -> rgb.setColour(GobildaRGBIndicatorHelper.Colour.RED)
                }
            }
        }

        wasIntaking = isIntaking
    }

    override fun end(interrupted: Boolean) {
        inSubsystem.stop()
    }
}
