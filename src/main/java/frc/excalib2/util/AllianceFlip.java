package frc.excalib2.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * Alliance-relative field geometry with <b>rotational</b> (180°) symmetry: values are
 * authored in blue-alliance coordinates and flipped when on red.
 *
 * <p>Field dimensions are injected once at startup ({@link #configure}) so the library
 * stays season-independent. (Port of ExcaLib v1 {@code AllianceUtils} semantics.)
 */
public final class AllianceFlip {
    private static double fieldLengthMeters = 16.54;
    private static double fieldWidthMeters = 8.07;

    private AllianceFlip() {
    }

    /** Sets this season's field dimensions. Call once before use. */
    public static void configure(double fieldLength, double fieldWidth) {
        fieldLengthMeters = fieldLength;
        fieldWidthMeters = fieldWidth;
    }

    public static double fieldLength() {
        return fieldLengthMeters;
    }

    public static double fieldWidth() {
        return fieldWidthMeters;
    }

    /** True when alliance is blue or unknown (defaults blue, like pre-match). */
    public static boolean isBlue() {
        return DriverStation.getAlliance()
                .map(alliance -> alliance == DriverStation.Alliance.Blue)
                .orElse(true);
    }

    /** Rotates a blue-frame translation 180° about the field center. */
    public static Translation2d flip(Translation2d translation) {
        return new Translation2d(
                fieldLengthMeters - translation.getX(),
                fieldWidthMeters - translation.getY());
    }

    public static Rotation2d flip(Rotation2d rotation) {
        return rotation.rotateBy(Rotation2d.k180deg);
    }

    public static Pose2d flip(Pose2d pose) {
        return new Pose2d(flip(pose.getTranslation()), flip(pose.getRotation()));
    }

    /** Returns the blue-frame value on blue, the flipped value on red. */
    public static Translation2d apply(Translation2d blueFrame) {
        return isBlue() ? blueFrame : flip(blueFrame);
    }

    public static Rotation2d apply(Rotation2d blueFrame) {
        return isBlue() ? blueFrame : flip(blueFrame);
    }

    public static Pose2d apply(Pose2d blueFrame) {
        return isBlue() ? blueFrame : flip(blueFrame);
    }

    /**
     * A field translation authored in blue-frame coordinates; {@link #get()} returns the
     * current alliance's version (port of ExcaLib v1 {@code AllianceUtils.AllianceTranslation}).
     */
    public static class AllianceTranslation {
        private final Translation2d blueFrame;

        public AllianceTranslation(double blueX, double blueY) {
            this.blueFrame = new Translation2d(blueX, blueY);
        }

        public AllianceTranslation(Translation2d blueFrame) {
            this.blueFrame = blueFrame;
        }

        public AllianceTranslation() {
            this(new Translation2d());
        }

        /** Alliance-corrected translation. */
        public Translation2d get() {
            return apply(blueFrame);
        }

        public Translation2d getBlue() {
            return blueFrame;
        }
    }

    /**
     * A field pose authored in blue-frame coordinates; {@link #get()} returns the current
     * alliance's version (port of ExcaLib v1 {@code AllianceUtils.AlliancePose}).
     */
    public static class AlliancePose {
        private final Pose2d blueFrame;

        public AlliancePose(double blueX, double blueY, double blueRotationRadians) {
            this.blueFrame = new Pose2d(blueX, blueY, new Rotation2d(blueRotationRadians));
        }

        public AlliancePose(Pose2d blueFrame) {
            this.blueFrame = blueFrame;
        }

        public AlliancePose() {
            this(new Pose2d());
        }

        /** Alliance-corrected pose. */
        public Pose2d get() {
            return apply(blueFrame);
        }

        public Pose2d getBlue() {
            return blueFrame;
        }
    }
}
