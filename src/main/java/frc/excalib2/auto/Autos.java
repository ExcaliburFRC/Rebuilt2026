package frc.excalib2.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.Map;

/**
 * PathPlanner auto glue: named-command registration + a lazily-built auto chooser.
 *
 * <p>Register named commands <b>before</b> building the chooser — PathPlanner substitutes
 * silent no-ops for unregistered names (this exact failure shipped once; never again).
 */
public final class Autos {
    private Autos() {
    }

    /** Registers every named command referenced by the .auto files. */
    public static void registerNamedCommands(Map<String, Command> commands) {
        commands.forEach(NamedCommands::registerCommand);
    }

    /**
     * Builds and publishes an auto chooser from all autos in {@code deploy/pathplanner/autos}.
     * Default option is "Do Nothing" (never crash on an empty selection).
     */
    public static SendableChooser<Command> buildChooser() {
        SendableChooser<Command> chooser = new SendableChooser<>();
        chooser.setDefaultOption("Do Nothing", Commands.none());
        for (String autoName : AutoBuilder.getAllAutoNames()) {
            chooser.addOption(autoName, AutoBuilder.buildAuto(autoName));
        }
        SmartDashboard.putData("Auto Chooser", chooser);
        return chooser;
    }
}
