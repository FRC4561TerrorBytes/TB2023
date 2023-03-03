package frc.robot;

/**
 * A singleton instance of this class holds the current robot state with regard
 * to game objectives such as the current game piece mode.
 */
public class GameState {
  /** The singleton instance. */
  private static final GameState INSTANCE = new GameState();

  /** The available game piece types. */
  public enum GamePiece {
    CONE,
    CUBE;
  }

  /**
   * The current game piece type we are trying to handle and an indicator of we
   * have one. Note that this information may not be accurate during autonomous.
   */
  private GamePiece m_gamePieceDesired = GamePiece.CUBE;
  private boolean m_gamePieceHeld = false;

  /** The possible states of an active April tag approach. */
  public enum CenteredState {
    CENTERED,
    PARTIAL,
    NOTCENTERED,
    NONE;
  }

  /** The current state of the active April tag approach. */
  private CenteredState m_centeredState = CenteredState.NONE;

  /** Private to only allow a single instance. */
  private GameState() {
  }

  /**
   * Clients obtain the single instance here.
   * 
   * @return the single {@link GameState} instance.
   */
  public static GameState getInstance() {
    return INSTANCE;
  }

  /**
   * @return the type of {@link GamePiece} we are currently handling. Never
   *         returns null.
   */
  public GamePiece getGamePieceDesired() {
    return m_gamePieceDesired;
  }

  /**
   * Changes the type of {@link GamePiece} we are currently handling.
   * 
   * @param piece the new type of game piece. Null is set as cube.
   */
  public void setGamePieceDesired(final GamePiece piece) {
    m_gamePieceDesired = piece == null ? GamePiece.CUBE : piece;
  }

  /**
   * @return true if we are holding a {@link GamePiece} and false if not.
   */
  public boolean isGamePieceHeld() {
    return m_gamePieceHeld;
  }

  /**
   * Sets the value returned by {@link #isGamePieceHeld()}. Typically set to true
   * after getting a game piece and to false after scoring it.
   * 
   * @param pieceHeld the new value for {@link #isGamePieceHeld()}.
   */
  public void setGamePieceHeld(boolean pieceHeld) {
    m_gamePieceHeld = pieceHeld;
  }

  /**
   * @return the current state of the active April tag approach. Never returns
   *         null.
   */
  public CenteredState getCenteredState() {
    return m_centeredState;
  }

  /**
   * Sets the result of {@link #getCenteredState()}.
   * 
   * @param state the current state of the active April tag approach.
   */
  public void setCenteredState(final CenteredState state) {
    m_centeredState = state == null ? CenteredState.NONE : state;
  }
}
