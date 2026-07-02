package frc.excalib2.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.Supplier;

/**
 * Field-zone triggers: declarative "is the robot in this area" conditions for superstructure
 * goal selection (alliance zone, trench, neutral zone, ...).
 *
 * <p>Rectangles are authored in blue-frame coordinates; {@code alliance*} variants flip on
 * red via {@link AllianceFlip}. (Concept credit: MA5951 {@code DeafultSuperStructure}.)
 */
public final class Zones {
    private Zones() {
    }

    /** Trigger: pose is inside the fixed (field-frame) rectangle. */
    public static Trigger inRectangle(Supplier<Pose2d> pose, Rectangle2d rectangle) {
        return new Trigger(() -> rectangle.contains(pose.get().getTranslation()));
    }

    /** Trigger: pose is inside the rectangle authored in blue-frame, flipped on red. */
    public static Trigger inAllianceRectangle(Supplier<Pose2d> pose, Rectangle2d blueFrameRectangle) {
        return new Trigger(() -> {
            Translation2d position = pose.get().getTranslation();
            Translation2d testPoint = AllianceFlip.isBlue() ? position : AllianceFlip.flip(position);
            return blueFrameRectangle.contains(testPoint);
        });
    }
}
