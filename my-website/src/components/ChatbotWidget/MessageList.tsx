import React, { useEffect, useRef } from 'react';
import type { Message } from './ChatbotWidget';
import styles from './styles.module.css';

interface MessageListProps {
  messages: Message[];
  isLoading: boolean;
}

const MessageList: React.FC<MessageListProps> = ({ messages, isLoading }) => {
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Auto-scroll to bottom when new messages arrive
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  return (
    <div className={styles.messageList}>
      {messages.length === 0 && !isLoading && (
        <div className={styles.emptyState}>
          <p>👋 Hi! I'm your textbook assistant.</p>
          <p>Ask me anything about Physical AI and Humanoid Robotics!</p>
        </div>
      )}

      {messages.map((message) => (
        <div
          key={message.id}
          className={`${styles.message} ${styles[message.role]}`}
        >
          <div className={styles.messageContent}>
            {/* Render message with markdown-style formatting */}
            {message.content.split('\n').map((line, idx) => (
              <p key={idx}>{line || <br />}</p>
            ))}

            {/* Render citations if present */}
            {message.citations && message.citations.length > 0 && (
              <div className={styles.citations}>
                <p className={styles.citationsTitle}>📚 Sources:</p>
                <ul>
                  {message.citations.map((citation, idx) => (
                    <li key={idx}>
                      <a
                        href={citation.url}
                        target="_blank"
                        rel="noopener noreferrer"
                        className={styles.citationLink}
                      >
                        {citation.chapter_title}: {citation.heading}
                      </a>
                    </li>
                  ))}
                </ul>
              </div>
            )}
          </div>

          <div className={styles.messageTimestamp}>
            {message.timestamp.toLocaleTimeString([], {
              hour: '2-digit',
              minute: '2-digit'
            })}
          </div>
        </div>
      ))}

      {isLoading && (
        <div className={`${styles.message} ${styles.assistant}`}>
          <div className={styles.messageContent}>
            <div className={styles.loadingDots}>
              <span></span>
              <span></span>
              <span></span>
            </div>
          </div>
        </div>
      )}

      <div ref={messagesEndRef} />
    </div>
  );
};

export default MessageList;
