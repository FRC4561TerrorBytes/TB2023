package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GameState;
import frc.robot.GameState.CenteredState;
import frc.robot.GameState.GamePiece;

/**
 * The LED subsystem observes the {@link GameState} and sets the human player
 * and driver feedback LEDs to signal important game state changes. While in the
 * loading zone at the far end of the field, the half of the strip on the front
 * of the arm is used to signal the human players and the half of the strip on
 * the back of the arm is used to signal the drivers.
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
    private boolean m_lastGamePieceHeldApplied = false;

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
     * Sets the human player, far end double substation facing, LEDs to the
     * specified RGB color.
     * 
     * @param r the r value [0-255]
     * @param g the g value [0-255]
     * @param b the b value [0-255]
     */
    private void setHumanPlayerLEDs(int r, int g, int b) {
        for (var i = 65; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, r, g, b);
        }
        m_led.setData(m_ledBuffer);
    }

    /**
     * Sets the driver, when in the far end loading zone, LEDs to the specified RGB
     * color.
     * 
     * @param r the r value [0-255]
     * @param g the g value [0-255]
     * @param b the b value [0-255]
     */
    private void setDriverSideLEDs(int r, int g, int b) {
        for (var i = 0; i < 65; i++) {
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
        // First handle current game piece type for human player.
        final GamePiece gamePiece = m_gameState.getGamePieceDesired();
        // Only apply LED update if it has not already been done.
        if (m_lastGamePieceApplied != gamePiece) {
            m_lastGamePieceApplied = gamePiece;
            switch (gamePiece) {
                case CONE:
                    setHumanPlayerLEDs(140, 40, 0);
                    break;
                case CUBE:
                    setHumanPlayerLEDs(62, 13, 115);
                    break;
                default:
                    setHumanPlayerLEDs(0, 0, 0);
                    break;
            }
        }

        // Next set the at loading station driver side.
        // Having possession overrides all else.
        if (m_gameState.isGamePieceHeld()) {
            // Only apply LED update if it has not already been done.
            if (!m_lastGamePieceHeldApplied) {
                m_lastGamePieceHeldApplied = true;
                setDriverSideLEDs(255, 255, 255);
            }
        } else {
            // Make sure next game piece held transition gets applied.
            m_lastGamePieceHeldApplied = false;
            final CenteredState centeredState = m_gameState.getCenteredState();
            // Only apply LED update if it has not already been done.
            if (m_lastCenteredStateApplied != centeredState) {
                m_lastCenteredStateApplied = centeredState;
                switch (centeredState) {
                    case NOTCENTERED:
                        setDriverSideLEDs(255, 0, 0);
                        break;
                    case PARTIAL:
                        setDriverSideLEDs(251, 156, 0);
                        break;
                    case CENTERED:
                        setDriverSideLEDs(0, 255, 0);
                        break;
                    default:
                        setDriverSideLEDs(0, 0, 0);
                        break;
                }
            }
        }
    }
}
