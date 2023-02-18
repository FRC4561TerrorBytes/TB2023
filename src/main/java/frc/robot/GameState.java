package frc.robot;
public class GameState {
    private static final GameState INSTANCE = new GameState();

    public enum GamePiece{
        CONE,
        CUBE,
        NONE;
    }

    private GamePiece m_gamePieceDesired = GamePiece.NONE;
    private GamePiece m_gamePieceHeld = GamePiece.NONE;


    public enum CenteredState{
        CENTERED,
        PARTIAL,
        NOTCENTERED,
        NONE;
    }

    private CenteredState m_centeredState = CenteredState.NONE;

    private GameState(){

    }

    public static GameState getInstance(){
        return INSTANCE;
    }

    public GamePiece getGamePieceDesired(){
        return m_gamePieceDesired;
    }

    public void setGamePieceDesired(GamePiece piece){
        m_gamePieceDesired = piece;
    }


    public GamePiece getGamePieceHeld(){
        return m_gamePieceHeld;
    }
    
    public void setGamePieceHeld(GamePiece piece){
        m_gamePieceHeld = piece;
    }

    public CenteredState getCenteredState(){
        return m_centeredState;
    }

    public void setCenteredState(CenteredState state){
        m_centeredState = state;
    }
}
