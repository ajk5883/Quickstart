package org.firstinspires.ftc.teamcode.pedroPathing.Auton;

/**
 * Describes the role of the destination pose in a {@link PathStep}.
 *
 * <p>{@link AutonBase} reads the pose type when the follower finishes a path and
 * triggers the corresponding subsystem actions:
 *
 * <table>
 *   <tr><th>Type</th><th>Action on arrival</th></tr>
 *   <tr><td>STARTING</td>
 *       <td>Keep shooter running between shots; no path to follow.
 *           Immediately advances to the next step.</td></tr>
 *   <tr><td>SCORING</td>
 *       <td>Stop intake, open gate, run timed shoot sequence, then wait for the
 *           shoot-window timer before advancing.</td></tr>
 *   <tr><td>PICKUP_START</td>
 *       <td>Start intake; immediately advance to the next step (sweep begins).</td></tr>
 *   <tr><td>PICKUP_END</td>
 *       <td>Stop intake; immediately advance to the next step.</td></tr>
 *   <tr><td>PARKING</td>
 *       <td>Stop all subsystems; terminal state.</td></tr>
 * </table>
 */
public enum PoseType {

    /** Initial robot pose (no path to follow). Shooter kept running between shots. */
    STARTING,

    /** Robot scores from this pose. Gate opens; timed shoot sequence runs; then wait. */
    SCORING,

    /**
     * Entry point of a ball-pickup sweep.
     * Intake turns on; robot immediately continues along to {@link #PICKUP_END}.
     */
    PICKUP_START,

    /**
     * Exit point of a ball-pickup sweep.
     * Intake turns off; robot immediately continues to the next step.
     */
    PICKUP_END,

    /** Final park position. All subsystems are stopped. Terminal state. */
    PARKING
}
