package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GameState;
import frc.robot.GameState.CenteredState;
import frc.robot.GameState.GamePiece;

/**
 * The LED subsystem observes the {@link GameState} and sets the human player
 * and driver feedback LEDs to signal important game state changes. While not
 * holding a game piece, that is the next critical activity is at the loading
 * zone at the far end of the field, the half of the strip on the front of the
 * arm is used to signal the human players and the half of the strip on
 * the back of the arm is used to signal the drivers. When holding a game piece,
 * that is the next critical activity is in the community, the front LEDs are
 * used to signal the drivers.
 * 
 * <p>
 * Note that the two strips on the robot arm are driven as one strip. Both
 * strips will always show the same pattern and colors. This is due to
 * restrictions in the {@link AddressableLED} class and this underlying roboRio
 * hardware.
 * </p>
 */
public class LEDSubsystem extends SubsystemBase {
    /** The connection for driving the LEDs. */
    private final AddressableLED m_led = new AddressableLED(9);
    /** A buffer for individually addressing each LED. */
    private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(144);

    /** Cached {@link GameState} singleton. */
    private final GameState m_gameState;

    /** Avoid unneeded and potentially costly game piece LED updates. */
    private GamePiece m_lastGamePieceApplied = null;
    /** Avoid unneeded and potentially costly centered state LED updates. */
    private CenteredState m_lastCenteredStateApplied = null;
    /** Avoid unneeded and potentially costly game piece held LED updates. */
    private Boolean m_lastGamePieceHeldApplied = null;

    /**
     * Creates a new {@link LEDSubsystem}.
     */
    public LEDSubsystem() {
        m_gameState = GameState.getInstance();
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    /**
     * Sets the back half, on the back of the arm, LEDs to the specified RGB color.
     * 
     * @param r the r value [0-255]
     * @param g the g value [0-255]
     * @param b the b value [0-255]
     */
    private void setBackHalfLED(int r, int g, int b) {
        for (var i = 0; i < 65; i++) {
            m_ledBuffer.setRGB(i, r, g, b);
        }
        m_led.setData(m_ledBuffer);
    }

    /**
     * Sets the front half, on the front of the arm, LEDs to the specified RGB
     * color.
     * 
     * @param r the r value [0-255]
     * @param g the g value [0-255]
     * @param b the b value [0-255]
     */
    private void setFrontHalfLED(int r, int g, int b) {
        for (var i = 65; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, r, g, b);
        }
        m_led.setData(m_ledBuffer);
    }

    /**
     * This implementation monitors the {@link GameState} and sets the LEDs
     * accordingly.
     * 
     * <p>
     * {@inheritDoc}
     */
    @Override
    public void periodic() {
        // Holding a game piece or not drives the decisions of where to place feedback.
        final boolean isGamePieceHeld = m_gameState.isGamePieceHeld();

        // If the game piece held state has changed, make sure all feedback gets applied
        // this cycle.
        if ((m_lastGamePieceHeldApplied == null)
                || (m_lastGamePieceHeldApplied.booleanValue() != isGamePieceHeld)) {
            m_lastGamePieceApplied = null;
            m_lastCenteredStateApplied = null;
            m_lastGamePieceHeldApplied = null;
        }

        if (isGamePieceHeld) {
            // Show game piece held since we have one and always on back.
            // Only apply LED update if it has not already been done.
            if (((m_lastGamePieceHeldApplied == null)
                    || (m_lastGamePieceHeldApplied.booleanValue() != isGamePieceHeld))) {
                m_lastGamePieceHeldApplied = isGamePieceHeld;
                setBackHalfLED(255, 255, 255);
            }
        } else {
            // Nothing to apply for not held, but match last applied state.
            m_lastGamePieceHeldApplied = Boolean.FALSE;

            // Handle current game piece type for human player.
            // Only shown if not holding a game piece and always on front.
            final GamePiece gamePiece = m_gameState.getGamePieceDesired();
            // Only apply LED update if it has not already been done.
            if (m_lastGamePieceApplied != gamePiece) {
                m_lastGamePieceApplied = gamePiece;
                switch (gamePiece) {
                    case CONE:
                        setFrontHalfLED(140, 40, 0);
                        break;
                    case CUBE:
                        setFrontHalfLED(62, 13, 115);
                        break;
                    default:
                        // Bug in GameState if we get here.
                        setFrontHalfLED(0, 0, 0);
                        break;
                }
            }
        }

        // This state always shows up on LEDs but may be front or back.
        // It will be the side not used above.
        final CenteredState centeredState = m_gameState.getCenteredState();
        // Only apply LED update if it has not already been done.
        if (m_lastCenteredStateApplied != centeredState) {
            m_lastCenteredStateApplied = centeredState;
            switch (centeredState) {
                case NOTCENTERED:
                    if (isGamePieceHeld) {
                        setFrontHalfLED(255, 0, 0);
                    } else {
                        setBackHalfLED(255, 0, 0);
                    }
                    break;
                case PARTIAL:
                    if (isGamePieceHeld) {
                        setFrontHalfLED(251, 156, 0);
                    } else {
                        setBackHalfLED(251, 156, 0);
                    }
                    break;
                case CENTERED:
                    if (isGamePieceHeld) {
                        setFrontHalfLED(0, 255, 0);
                    } else {
                        setBackHalfLED(0, 255, 0);
                    }
                    break;
                default:
                    // NONE or a bug in GameState would get us here.
                    if (isGamePieceHeld) {
                        setFrontHalfLED(0, 0, 0);
                    } else {
                        setBackHalfLED(0, 0, 0);
                    }
                    break;
            }
        }
    }
}
