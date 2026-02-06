package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.shooter.OuttakeSubsystem

class IntakeCommand(
    private val inSubsystem: IntakeSubsystem,
    private val out: OuttakeSubsystem,
    private val driver: GamepadEx
) : CommandBase() {


    init {
        addRequirements(inSubsystem, out)
    }

    override fun execute() {
        val leftTrigger = driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)
        val rightTrigger = driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)

        if (leftTrigger > 0.1 || rightTrigger > 0.1) {
            inSubsystem.intake(leftTrigger - rightTrigger)
        } else {
            inSubsystem.stop()
        }
    }

    override fun end(interrupted: Boolean) {
        inSubsystem.stop()
    }
}
