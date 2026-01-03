/**
 * ChatbotWidget - Floating chat interface for textbook Q&A
 *
 * Connects to FastAPI backend (T027: POST /chat endpoint)
 * Maps to: FR-007 (Embedded chatbot), SC-002 (<3s response time)
 *
 * Features:
 * - Session persistence across navigation
 * - Full-textbook RAG context
 * - Clickable citations to source material
 * - Rate limiting error handling
 * - Response time display
 *
 * Configuration:
 * - Set REACT_APP_API_URL environment variable for backend URL
 * - Default: http://localhost:8000 (development)
 * - Production: Set to Render backend URL (per T035)
 */

import React, { useState, useCallback } from 'react';
import ChatInput from './ChatInput';
import MessageList from './MessageList';
import styles from './styles.module.css';

export interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  citations?: Citation[];
  timestamp: Date;
  response_time_ms?: number;
}

export interface Citation {
  chunk_id: string;
  chapter_title: string;
  heading: string;
  url: string;
  chunk_preview?: string;
}

interface ChatbotWidgetProps {
  apiBaseUrl?: string;
}

const ChatbotWidget: React.FC<ChatbotWidgetProps> = ({
  apiBaseUrl = typeof window !== 'undefined'
    ? (window as any).CHATBOT_API_URL || 'https://mraiss-ai-course-book.hf.space'
    : 'https://mraiss-ai-course-book.hf.space'
}) => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [sessionId, setSessionId] = useState<string | null>(null);
  const [isLoading, setIsLoading] = useState(false);

  const toggleChat = useCallback(() => {
    setIsOpen(prev => !prev);
  }, []);

  const sendMessage = useCallback(async (messageText: string) => {
    if (!messageText.trim()) return;

    // Add user message to UI immediately
    const userMessage: Message = {
      id: Date.now().toString(),
      role: 'user',
      content: messageText,
      timestamp: new Date()
    };

    setMessages(prev => [...prev, userMessage]);
    setIsLoading(true);

    try {
      // Call chatbot API (per T027: POST /chat endpoint)
      const response = await fetch(`${apiBaseUrl}/api/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          session_id: sessionId,
          message: messageText,
          context: {
            mode: 'full_textbook',
            selected_module_ids: [],
            selected_chapter_ids: []
          }
        })
      });

      if (!response.ok) {
        // Handle rate limiting (429)
        if (response.status === 429) {
          const errorData = await response.json();
          throw new Error(errorData.detail || 'Rate limit exceeded. Please try again later.');
        }
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();

      // Update session ID if new session was created
      if (data.session_id) {
        setSessionId(data.session_id);
      }

      // Add assistant message to UI with citations and response time
      const assistantMessage: Message = {
        id: (Date.now() + 1).toString(),
        role: 'assistant',
        content: data.message,
        citations: data.citations || [],
        timestamp: new Date(),
        response_time_ms: data.response_time_ms
      };

      setMessages(prev => [...prev, assistantMessage]);
    } catch (error) {
      console.error('Error sending message:', error);

      // Add error message to UI
      const errorMessage: Message = {
        id: (Date.now() + 1).toString(),
        role: 'assistant',
        content: error instanceof Error
          ? error.message
          : 'Sorry, I encountered an error processing your request. Please try again.',
        timestamp: new Date()
      };

      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  }, [apiBaseUrl, sessionId]);

  return (
    <>
      {/* Floating chat button */}
      <button
        className={styles.chatButton}
        onClick={toggleChat}
        aria-label={isOpen ? 'Close chat' : 'Open chat'}
        title={isOpen ? 'Close chat' : 'Ask a question about the textbook'}
      >
        {isOpen ? '✕' : '💬'}
      </button>

      {/* Chat widget */}
      {isOpen && (
        <div className={styles.chatWidget}>
          <div className={styles.chatHeader}>
            <h3>Textbook Assistant</h3>
            <button
              className={styles.closeButton}
              onClick={toggleChat}
              aria-label="Close chat"
            >
              ✕
            </button>
          </div>

          <MessageList messages={messages} isLoading={isLoading} />

          <ChatInput onSendMessage={sendMessage} disabled={isLoading} />
        </div>
      )}
    </>
  );
};

export default ChatbotWidget;
