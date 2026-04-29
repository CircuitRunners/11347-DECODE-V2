package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakePivot
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.transfer.ColourZoneDetection
import org.firstinspires.ftc.teamcode.support.GobildaRGBIndicatorHelper
import kotlin.math.abs

class IntakeCommand(
    private val inSubsystem : IntakeSubsystem,
    private val driver : GamepadEx,
    private val pivot : IntakePivot
) : CommandBase() {
    private var wasIntaking = false

    init {
        addRequirements(inSubsystem)
    }

    override fun execute() {
        val leftTrigger = driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)
        val rightTrigger = driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)
        val isIntaking = (leftTrigger > 0.1 || rightTrigger > 0.1)

        if (isIntaking) {
            inSubsystem.intake(leftTrigger - rightTrigger)
            if (abs(leftTrigger) > abs(rightTrigger)) {
                pivot.setPivotIntaking()
            } else {
                pivot.setPivotMax()
            }
        } else {
            inSubsystem.stop()
        }

        wasIntaking = isIntaking
    }

    override fun end(interrupted: Boolean) {
        inSubsystem.stop()
    }
}
