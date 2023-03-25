package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.GameState;
import frc.robot.GameState.GamePiece;

/**
 * The LED subsystem observes the {@link GameState} and sets the human player
 * and driver feedback LEDs to signal important game state changes. The LEDs on
 * the front of the arm signal the game piece being held or desired. The LEDs on
 * the back of the arm are blue when a game piece is held and off, otherwise.
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

  /** Used to create onTrue and onFalse edges for enum change. */
  private GamePiece m_lastGamePieceDesiredApplied = null;

  /**
   * Creates a new {@link LEDSubsystem}.
   */
  public LEDSubsystem() {
    m_gameState = GameState.getInstance();
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
    createGameStateBindings();
    // Initialize LEDs to starting game state.
    setFrontToGamePiece(m_gameState.getGamePieceDesired());
    if (m_gameState.isGamePieceHeld()) {
      setBackHalfLED(0, 0, 255);
    }
  }

  /**
   * Create and bind triggers for {@link GameState} change to LED update.
   */
  private void createGameStateBindings() {
    // Trigger for game piece held changes.
    final Trigger gamePieceHeldTrigger = new Trigger(m_gameState::isGamePieceHeld);
    // On game piece held, show blue on back of arm.
    gamePieceHeldTrigger.onTrue(new InstantCommand(() -> {
      setBackHalfLED(0, 0, 255);
    }));
    // On game piece gone, turn off back of arm.
    gamePieceHeldTrigger.onFalse(new InstantCommand(() -> {
      setBackHalfLED(0, 0, 0);
    }));

    // Change the game piece indication of the front of the arm
    // anytime it changes in game state.
    final Trigger gamePieceDesiredTrigger = new Trigger(
        () -> (m_gameState.getGamePieceDesired() != m_lastGamePieceDesiredApplied));
    gamePieceDesiredTrigger
        .onTrue(new InstantCommand(() -> {
          final GamePiece gamePiece = m_gameState.getGamePieceDesired();
          setFrontToGamePiece(gamePiece);
          m_lastGamePieceDesiredApplied = gamePiece;
        }));
  }

  /**
   * Sets the LEDs on the front of the arm to match the parameter.
   * 
   * @param gamePiece the {@link GamePiece} being held or needed.
   */
  private void setFrontToGamePiece(final GamePiece gamePiece) {
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
}
