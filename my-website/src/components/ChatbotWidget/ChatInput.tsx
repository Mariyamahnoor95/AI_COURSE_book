import React, { useState, useCallback, KeyboardEvent } from 'react';
import styles from './styles.module.css';

interface ChatInputProps {
  onSendMessage: (message: string) => void;
  disabled?: boolean;
}

const ChatInput: React.FC<ChatInputProps> = ({ onSendMessage, disabled = false }) => {
  const [inputValue, setInputValue] = useState('');

  const handleSend = useCallback(() => {
    if (inputValue.trim() && !disabled) {
      onSendMessage(inputValue.trim());
      setInputValue('');
    }
  }, [inputValue, disabled, onSendMessage]);

  const handleKeyPress = useCallback((e: KeyboardEvent<HTMLTextAreaElement>) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSend();
    }
  }, [handleSend]);

  return (
    <div className={styles.chatInputContainer}>
      <textarea
        className={styles.chatInput}
        value={inputValue}
        onChange={(e) => setInputValue(e.target.value)}
        onKeyPress={handleKeyPress}
        placeholder="Ask a question about the textbook..."
        disabled={disabled}
        rows={2}
        aria-label="Chat message input"
      />
      <button
        className={styles.sendButton}
        onClick={handleSend}
        disabled={disabled || !inputValue.trim()}
        aria-label="Send message"
      >
        {disabled ? '⏳' : '➤'}
      </button>
    </div>
  );
};

export default ChatInput;
