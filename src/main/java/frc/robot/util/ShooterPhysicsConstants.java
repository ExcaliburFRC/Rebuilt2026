package frc.robot.util;

import edu.wpi.first.math.util.Units;

public final class ShooterPhysicsConstants {

    private ShooterPhysicsConstants() {}

    /* Field */
    public static final double HUB_HEIGHT_METERS = Units.inchesToMeters(72.0);
    public static final double HUB_ENTRY_OFFSET_METERS = 0.08;

    /* Shooter */
    public static final double SHOOTER_EXIT_HEIGHT_METERS = Units.inchesToMeters(40.0); // FILL
    public static final double MAIN_ROLLER_RADIUS_METERS = Units.inchesToMeters(2.0);

    /* Physics */
    public static final double GRAVITY = 9.80665;

    /* Efficiency */
    public static final double COMPRESSION_EFFICIENCY = 0.85;
    public static final double SLIP_EFFICIENCY = 0.90;

    /* Drag */
    public static final double DRAG_COEFFICIENT = 0.06;

    /* Magnus */
    public static final double MAGNUS_COEFFICIENT = 0.15;
    public static final double FUEL_RADIUS_METERS = Units.inchesToMeters(5.91 / 2.0);

    /* Entry angle interpolation */
    public static final double MIN_DISTANCE = 1.5;
    public static final double MAX_DISTANCE = 6.0;

    public static final double ENTRY_ANGLE_CLOSE_RAD = Units.degreesToRadians(50.0);
    public static final double ENTRY_ANGLE_FAR_RAD   = Units.degreesToRadians(38.0);

    /* Hood limits */
    public static final double MIN_HOOD_ANGLE_RAD = Units.degreesToRadians(20.0);
    public static final double MAX_HOOD_ANGLE_RAD = Units.degreesToRadians(65.0);
}
