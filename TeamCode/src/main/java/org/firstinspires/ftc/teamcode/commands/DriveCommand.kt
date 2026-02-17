package org.firstinspires.ftc.teamcode.commands

import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.gamepad.GamepadEx
import org.firstinspires.ftc.teamcode.subsystems.drive.MecanumDrivebase

class DriveCommand(
    private val drive : MecanumDrivebase,
    private val driver : GamepadEx,
    private val alliance : Boolean
) : CommandBase() {
    init{
        addRequirements(drive)
    }

    override fun execute() {
        super.execute()
        val forwards = driver.leftY
        val translate = driver.leftX
        val rotate = driver.rightX

        drive.driveFieldRelative(
            forwards,
            translate,
            rotate,
            alliance)
    }

    override fun end(interrupted: Boolean) {
        drive.driveFieldRelative(0.0, 0.0, 0.0, alliance)
    }
}
