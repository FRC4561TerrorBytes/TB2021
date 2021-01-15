package frc.robot;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TableEntryListener;
import edu.wpi.first.wpilibj.DigitalOutput;

/**
 * A helper class with methods to get vision data from NetworkTables
 * @author Zane
 */
public class VisionData {

    private static NetworkTable table = NetworkTableInstance.getDefault().getTable("SmartDashboard");
    private static DigitalOutput LED_RING = new DigitalOutput(Constants.LED_RING_PORT);
    
    /**
     * TODO Return false if the pi has not pushed data in a while
     * @return True if vision is enabled and NetworkTable values have been created
     */
    public static boolean isReady() {
        return isEnabled() && table.getEntry("detected?").exists();
    }
    /**
     * @return True if the LED ring is on
     */
    public static boolean isEnabled() {
        return LED_RING.get();
    }
    /**
     * @param bool True to enable, false to disable vision
     */
    public static void setEnabled(boolean bool) {
        LED_RING.set(bool);
    }
    /**
     * Toggles whether vision is enabled or disabled
     */
    public static void toggle() {
        LED_RING.set(!isEnabled());
        Constants.VISION_ENABLE = LED_RING.get();
    }

    /**
     * @return True if a vision target has been detected
     */
    public static boolean isDetected() {
        return table.getEntry("detected?").getBoolean(false);
    }
    /**
     * @return The horizontal angle between the robot and center of vision target in degrees
     * Returns 0.0 by default if there is no data, check using {@link isReady()}
     */
    public static double getXAngle() {
        return table.getEntry("xAngle").getDouble(0.0);
    }
    /**
     * @return The horizonal distance between the robot and center of vision target in inches
     * Returns 0.0 by default if there is no data, check using {@link isReady()}
     */
    public static double getDistance() {
        return table.getEntry("distance").getDouble(0.0);
    }

    public static void setXAngleListener(TableEntryListener listener) {
        table.addEntryListener("xAngle", listener, EntryListenerFlags.kUpdate);
    }
    
}