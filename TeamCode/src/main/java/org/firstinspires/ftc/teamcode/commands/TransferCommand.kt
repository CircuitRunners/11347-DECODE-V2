//package org.firstinspires.ftc.teamcode.commands
//
//import com.arcrobotics.ftclib.command.CommandBase
//import com.arcrobotics.ftclib.gamepad.GamepadEx
//import com.arcrobotics.ftclib.gamepad.GamepadKeys
//import org.firstinspires.ftc.teamcode.subsystems.transfer.kicker
//
//class TransferCommand(
//    private var kickSubsystem: kicker,
//    private var driver: GamepadEx,
//) : CommandBase() {
//    private var blocking = true;
//
//    init {
//        addRequirements(kickSubsystem)
//    }
//
//    override fun execute() {
//        if (driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).get()) {
//            kickSubsystem.kickerTransferUP(servoNumber)
//        } else {
//            kickSubsystem.kickerDownPosition()
//        }
//    }
//
//    override fun end(interrupted: Boolean) {
//        kickSubsystem.kickerDownPosition()
//    }
//
//    override fun isFinished() =
//        !driver.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).get()
//}
