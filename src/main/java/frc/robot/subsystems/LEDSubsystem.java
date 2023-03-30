package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
  private GamePiece m_lastGamePieceDesiredApplied = GamePiece.CUBE;

  private Command m_cubeCycle = new LEDCycleFront(62, 13, 155)
      .andThen(new WaitCommand(0.25))
      .andThen(new ScheduleCommand(new InstantCommand(() -> setFrontHalfLED(62, 13, 155))));
  private Command m_coneCycle = new LEDCycleFront(140, 40, 0)
      .andThen(new WaitCommand(0.25))
      .andThen(new ScheduleCommand(new InstantCommand(() -> setFrontHalfLED(140, 40, 0))));

  /**
   * Creates a new {@link LEDSubsystem}.
   */
  public LEDSubsystem() {
    m_gameState = GameState.getInstance();
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
    createGameStateBindings();
  }

  // Initialize LEDs to starting game state.
  public void init() {
    final Command initCmd = new ScheduleCommand(new InstantCommand(() -> {
      setFrontToGamePiece(m_gameState.getGamePieceDesired());
      if (m_gameState.isGamePieceHeld()) {
        setBackHalfLED(0, 0, 255);
      }
    }));
    initCmd.schedule();
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
        m_coneCycle.schedule();
        break;
      case CUBE:
        m_cubeCycle.schedule();
        break;
      default:
        // Bug in GameState if we get here.
        setFrontHalfLED(0, 255, 0);
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

  /**
   * A nested command class used to cycle the game piece lights.
   */
  public class LEDCycleFront extends CommandBase {
    private final int m_r;
    private final int m_g;
    private final int m_b;
    private final Timer m_timer = new Timer();
    private int m_lightsOn = 65;

    /** Creates a new LEDCycleFront. */
    public LEDCycleFront(int r, int g, int b) {
      m_r = r;
      m_g = g;
      m_b = b;
      addRequirements(LEDSubsystem.this);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      m_timer.reset();
      m_timer.start();
      m_lightsOn = 65;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      for (var i = 65; i < m_ledBuffer.getLength(); i++) {
        if (i <= m_lightsOn) {
          m_ledBuffer.setRGB(i, m_r, m_g, m_b);
        } else {
          m_ledBuffer.setRGB(i, 0, 0, 0);
        }
      }
      m_led.setData(m_ledBuffer);

      m_lightsOn++;
      if (m_lightsOn >= m_ledBuffer.getLength()) {
        m_lightsOn = 65;
      }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      m_timer.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return m_timer.hasElapsed(7.0);
    }
  }
}
