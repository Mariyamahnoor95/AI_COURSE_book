/**
 * MessageList component for rendering chat messages with citations
 *
 * Integrates with ChatKit.js rendering patterns per ADR-0005
 * Maps to: FR-016 (ChatKit SDK integration), FR-020 (Citation formatting)
 *
 * Note: Full ChatKit.js integration pending SDK public release.
 * Current implementation follows ChatKit UI patterns for forward compatibility.
 */

import React, { useEffect, useRef } from 'react';
import type { Message } from './ChatbotWidget';
import styles from './styles.module.css';

interface MessageListProps {
  messages: Message[];
  isLoading: boolean;
}

/**
 * Renders markdown-style formatting in message content
 * TODO: Replace with ChatKit.js Markdown renderer when available
 */
const renderMessageContent = (content: string): React.ReactNode => {
  // Split by double newlines to identify paragraphs
  const paragraphs = content.split('\n\n');

  return paragraphs.map((paragraph, pIdx) => {
    // Handle code blocks (```language...```)
    if (paragraph.trim().startsWith('```')) {
      const codeMatch = paragraph.match(/```(\w+)?\n([\s\S]*?)```/);
      if (codeMatch) {
        const language = codeMatch[1] || 'text';
        const code = codeMatch[2];
        return (
          <pre key={pIdx} className={styles.codeBlock}>
            <code className={`language-${language}`}>{code}</code>
          </pre>
        );
      }
    }

    // Handle inline code (`code`)
    const parts = paragraph.split(/(`[^`]+`)/g);
    const formattedParts = parts.map((part, idx) => {
      if (part.startsWith('`') && part.endsWith('`')) {
        return <code key={idx} className={styles.inlineCode}>{part.slice(1, -1)}</code>;
      }

      // Handle bold (**text**)
      if (part.includes('**')) {
        const boldParts = part.split(/(\*\*[^*]+\*\*)/g);
        return boldParts.map((bp, bIdx) => {
          if (bp.startsWith('**') && bp.endsWith('**')) {
            return <strong key={`${idx}-${bIdx}`}>{bp.slice(2, -2)}</strong>;
          }
          return bp;
        });
      }

      return part;
    });

    // Split by single newlines within paragraph
    const lines = paragraph.split('\n');
    return (
      <p key={pIdx} className={styles.messageParagraph}>
        {lines.map((line, lIdx) => (
          <React.Fragment key={lIdx}>
            {lIdx > 0 && <br />}
            {line}
          </React.Fragment>
        ))}
      </p>
    );
  });
};

/**
 * Renders citation links with chapter/heading format
 * Per FR-020: 📚 **Sources:** [Chapter: Heading](URL)
 * Per SC-005: 90% of responses should include citations
 */
const renderCitations = (citations: Message['citations']): React.ReactNode => {
  if (!citations || citations.length === 0) {
    return null;
  }

  return (
    <div className={styles.citations}>
      <p className={styles.citationsTitle}>📚 <strong>Sources:</strong></p>
      <ul className={styles.citationList}>
        {citations.map((citation, idx) => (
          <li key={citation.chunk_id || idx} className={styles.citationItem}>
            <a
              href={citation.url}
              target="_blank"
              rel="noopener noreferrer"
              className={styles.citationLink}
              title={citation.chunk_preview || `${citation.chapter_title}: ${citation.heading}`}
            >
              {citation.chapter_title}: {citation.heading}
            </a>
          </li>
        ))}
      </ul>
    </div>
  );
};

const MessageList: React.FC<MessageListProps> = ({ messages, isLoading }) => {
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Auto-scroll to bottom when new messages arrive
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  return (
    <div className={styles.messageList}>
      {/* Empty state when no messages */}
      {messages.length === 0 && !isLoading && (
        <div className={styles.emptyState}>
          <p>👋 Hi! I'm your textbook assistant.</p>
          <p>Ask me anything about Physical AI and Humanoid Robotics!</p>
        </div>
      )}

      {/* Render messages with ChatKit-compatible structure */}
      {messages.map((message) => (
        <div
          key={message.id}
          className={`${styles.message} ${styles[message.role]}`}
          data-message-id={message.id}
          data-role={message.role}
        >
          {/* Message content with markdown rendering */}
          <div className={styles.messageContent}>
            {renderMessageContent(message.content)}

            {/* Citations (only for assistant messages) */}
            {message.role === 'assistant' && renderCitations(message.citations)}
          </div>

          {/* Message metadata */}
          <div className={styles.messageTimestamp}>
            {message.timestamp.toLocaleTimeString([], {
              hour: '2-digit',
              minute: '2-digit'
            })}
            {message.response_time_ms && (
              <span className={styles.responseTime}>
                {' '}• {(message.response_time_ms / 1000).toFixed(1)}s
              </span>
            )}
          </div>
        </div>
      ))}

      {/* Loading indicator with animated dots */}
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

      {/* Scroll anchor */}
      <div ref={messagesEndRef} />
    </div>
  );
};

export default MessageList;

/*
 * ChatKit.js Integration Notes (per ADR-0005):
 *
 * When ChatKit.js SDK becomes publicly available, replace this component with:
 *
 * import { ChatView, MessageRenderer } from '@openai/chatkit-react';
 *
 * <ChatView
 *   messages={messages}
 *   renderMessage={(msg) => (
 *     <MessageRenderer message={msg}>
 *       {msg.role === 'assistant' && <CitationRenderer citations={msg.citations} />}
 *     </MessageRenderer>
 *   )}
 *   isLoading={isLoading}
 * />
 *
 * Benefits:
 * - Official OpenAI UI components
 * - Built-in streaming support
 * - Optimized rendering performance
 * - Automatic accessibility compliance
 */
