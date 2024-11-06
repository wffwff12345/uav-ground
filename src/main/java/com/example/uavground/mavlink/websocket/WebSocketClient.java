package com.example.uavground.mavlink.websocket;

import com.example.uavground.mavlink.config.WebSocketClientConfig;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;
import org.springframework.web.socket.TextMessage;
import org.springframework.web.socket.WebSocketHandler;
import org.springframework.web.socket.WebSocketSession;
import org.springframework.web.socket.client.WebSocketConnectionManager;
import org.springframework.web.socket.client.standard.StandardWebSocketClient;

@Service
public class WebSocketClient {
    private final StandardWebSocketClient webSocketClient;
    private final WebSocketHandler webSocketHandler;
    private WebSocketConnectionManager connectionManager;

    @Autowired
    public WebSocketClient(StandardWebSocketClient webSocketClient, WebSocketHandler webSocketHandler) {
        this.webSocketClient = webSocketClient;
        this.webSocketHandler = webSocketHandler;
    }

    public void connect(String url) {
        connectionManager = new WebSocketConnectionManager(webSocketClient, webSocketHandler, url);
        connectionManager.start();
    }

    public void disconnect() {
        connectionManager.stop();
    }

    public void sendMessage(String message) {
        if (connectionManager != null && connectionManager.isConnected() == true) {
            WebSocketSession session = WebSocketClientConfig.session;
            if (session != null && session.isOpen()) {
                try {
                    session.sendMessage(new TextMessage(message));
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
        }
    }
}
