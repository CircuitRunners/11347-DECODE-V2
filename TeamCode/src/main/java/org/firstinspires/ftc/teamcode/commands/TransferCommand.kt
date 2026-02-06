package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import org.firstinspires.ftc.teamcode.subsystems.transfer.Kickers

/**
 * TransferCommand
 */
class TransferCommand(
    private val kickers: Kickers,
    private val driver: GamepadEx,
) : CommandBase() {

    init {
        addRequirements(kickers)
    }

    override fun execute() {
        when {
            driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).get() -> kickers.kickZoneOne()
            driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).get() -> kickers.kickZoneTwo()
            driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).get() -> kickers.kickZoneThree()
            else -> {
                kickers.resetZoneOne()
                kickers.resetZoneTwo()
                kickers.resetZoneThree()
            }
        }
    }

    override fun end(interrupted: Boolean) {
        kickers.resetZoneOne()
        kickers.resetZoneTwo()
        kickers.resetZoneThree()
    }

    override fun isFinished(): Boolean {
        val left = driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).get()
        val up = driver.getGamepadButton(GamepadKeys.Button.DPAD_UP).get()
        val right = driver.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).get()
        return !(left || up || right)
    }
}
