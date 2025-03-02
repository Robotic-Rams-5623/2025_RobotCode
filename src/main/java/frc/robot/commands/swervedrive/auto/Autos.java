// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmExtend;
import frc.robot.subsystems.ArmLength;
import frc.robot.subsystems.ArmTilt;

/** Add your docs here. */
public class Autos {

    public Command Open(ArmTilt tilt) {
        return new InstantCommand(() -> tilt.setSmartPosition(1), tilt);
    }
}
