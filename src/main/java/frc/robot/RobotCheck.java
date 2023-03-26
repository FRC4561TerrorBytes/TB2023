package frc.robot;

import java.io.IOException;
import java.net.Socket;

public class RobotCheck {
  private static boolean available(int port) {
    System.out.println("----------------Testing port " + port);
    Socket s = null;
    try {
      s = new Socket("localhost", port);
      System.out.println("----------------Port " + port + " is not available.");
      return false;
    } catch (IOException e) {
      System.out.println("----------------Port " + port + "is available.");
      return true;
    } finally {
      if (s != null) {
        try {
          s.close();
        } catch (IOException e) {
          throw new RuntimeException("You should handle this error", e);
        }
      }
    }
  }

  public static void main(String[] args) {
    int attempts = 0;
    boolean scanning = true;
    while (scanning) {

      if (!available(9)) {
        attempts++;
        if (attempts == 10) {
          System.out.println("Giving up!");
          scanning = false;
        } else {
          scanning = false;
        }
        try {
          Thread.sleep(2000);// 2 seconds
        } catch (InterruptedException ie) {
          ie.printStackTrace();
        }
      }
    }
  }
}
