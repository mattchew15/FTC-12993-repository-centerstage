package debugclient.src.com.company;

import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.net.ServerSocket;
import java.net.Socket;

public class TcpipListener implements Runnable {

    //static ServerSocket variable
    private static ServerSocket server;

    //this processes and deals with the messages
    private final MessageProcessing myMessageProcessor;


    private int listeningPort = 666;

    /**
     * Constructor
     * @param port the tcpip port we will send on. This has to match the android program
     */
    public TcpipListener(int port){
        listeningPort = port;
        myMessageProcessor = new MessageProcessing();
    }


    /**
     * This doesn't run on the main thread
     */
    @Override
    public void run() {
        try{
            server = new ServerSocket(listeningPort);
            while(true){
                System.out.println("Waiting for client request");
                //creating socket and waiting for client connection
                Socket socket = server.accept();
                //read from socket to ObjectInputStream object
                ObjectInputStream ois = new ObjectInputStream(socket.getInputStream());
                //convert ObjectInputStream object to String
                String message = (String) ois.readObject();

                myMessageProcessor.processMessage(message);

//                //create ObjectOutputStream object
//                ObjectOutputStream oos = new ObjectOutputStream(socket.getOutputStream());
//                //write object to Socket
//                oos.writeObject("Hi Client "+message);
                //close resources
                ois.close();
//                oos.close();
                socket.close();


                //terminate the server if client sends exit request
                if(message.equalsIgnoreCase("exit")) break;
            }



            System.out.println("Shutting down Socket server!!");
            //close the ServerSocket object
            server.close();



        }catch(IOException e){//we also need these catch clauses
            e.printStackTrace();
        } catch (ClassNotFoundException e) {
            e.printStackTrace();
        }
    }
}
