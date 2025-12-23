/**
 * Floating RAG Chatbot Component
 * A minimalist floating chat widget for the Physical AI & Humanoid Robotics book
 */

import React, { useState, useRef, useEffect } from 'react';
import ReactMarkdown from 'react-markdown';
import { useAuth } from '../../contexts/AuthContext';
import {
  sendChatQuery,
  getOrCreateSessionId,
  getConversationId,
  setConversationId,
  clearConversation,
  ChatApiError,
  type ChatMessage as ChatMessageType,
} from '../../services/chatApi';
import styles from './Chatbot.module.css';

interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  timestamp: Date;
  metadata?: any;
}

interface ChatbotProps {
  selectedText?: string;
  onSelectedTextUsed?: () => void;
}

export default function Chatbot({ selectedText, onSelectedTextUsed }: ChatbotProps = {}): JSX.Element {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [pendingSelection, setPendingSelection] = useState<string | null>(null);
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLInputElement>(null);

  // Get auth context to access user information
  const { user } = useAuth();

  // Handle selected text from parent
  useEffect(() => {
    if (selectedText && selectedText.trim()) {
      setPendingSelection(selectedText);
      setIsOpen(true); // Auto-open chatbot
      setInputValue(`About this text: "${selectedText.substring(0, 50)}${selectedText.length > 50 ? '...' : ''}"`);
    }
  }, [selectedText]);

  // Auto-scroll to bottom when new messages arrive
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  // Focus input when chat opens
  useEffect(() => {
    if (isOpen && inputRef.current) {
      inputRef.current.focus();
    }
  }, [isOpen]);

  const handleSendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    const userMessage: Message = {
      id: Date.now().toString(),
      role: 'user',
      content: inputValue.trim(),
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);
    setError(null);

    try {
      const sessionId = getOrCreateSessionId();
      const conversationId = getConversationId();

      // Build query with optional text selection
      const queryPayload: any = {
        question: userMessage.content,
        session_id: sessionId,
        conversation_id: conversationId || undefined,
        user_id: user?.id, // Include user ID for personalization if authenticated
      };

      // Add text selection if present (SAFE: optional feature)
      if (pendingSelection && pendingSelection.trim()) {
        queryPayload.text_selection = {
          text: pendingSelection,
          chapter_id: 'selected-content',
          start_offset: 0,
          end_offset: pendingSelection.length,
        };
        // Clear after use
        setPendingSelection(null);
        if (onSelectedTextUsed) onSelectedTextUsed();
      }

      const response: ChatMessageType = await sendChatQuery(queryPayload);

      // Save conversation ID for continuity
      setConversationId(response.conversation_id);

      const assistantMessage: Message = {
        id: response.message_id,
        role: 'assistant',
        content: response.response,
        timestamp: new Date(response.timestamp),
        metadata: response.metadata,
      };

      setMessages(prev => [...prev, assistantMessage]);
    } catch (err) {
      console.error('Chat error:', err);

      let errorMessage = 'Failed to send message. Please try again.';
      if (err instanceof ChatApiError) {
        errorMessage = err.message;
      }

      setError(errorMessage);

      // Add error message to chat
      const errorMsg: Message = {
        id: Date.now().toString(),
        role: 'assistant',
        content: `❌ ${errorMessage}`,
        timestamp: new Date(),
      };
      setMessages(prev => [...prev, errorMsg]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  const handleNewConversation = () => {
    clearConversation();
    setMessages([]);
    setError(null);
    setInputValue('');  // Clear input field
    setPendingSelection(null);  // Clear pending selection
    if (onSelectedTextUsed) onSelectedTextUsed();  // Clear parent state
  };

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  return (
    <div className={styles.chatbotContainer}>
      {/* Floating Button */}
      {!isOpen && (
        <button
          className={styles.chatButton}
          onClick={toggleChat}
          aria-label="Open chat"
          title="Ask questions about the book"
        >
          <svg
            width="24"
            height="24"
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
            strokeWidth="2"
            strokeLinecap="round"
            strokeLinejoin="round"
          >
            <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" />
          </svg>
        </button>
      )}

      {/* Chat Panel */}
      {isOpen && (
        <div className={styles.chatPanel}>
          {/* Header */}
          <div className={styles.chatHeader}>
            <div>
              <h3>Book Assistant</h3>
              <p>Ask me anything about Physical AI & Humanoid Robotics</p>
            </div>
            <div className={styles.headerActions}>
              <button
                onClick={handleNewConversation}
                className={styles.newChatButton}
                title="New conversation"
              >
                <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
                  <path d="M1 4v6h6M23 20v-6h-6" />
                  <path d="M20.49 9A9 9 0 0 0 5.64 5.64L1 10m22 4l-4.64 4.36A9 9 0 0 1 3.51 15" />
                </svg>
              </button>
              <button
                onClick={toggleChat}
                className={styles.closeButton}
                aria-label="Close chat"
              >
                <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                  <path d="M18 6L6 18M6 6l12 12" />
                </svg>
              </button>
            </div>
          </div>

          {/* Messages */}
          <div className={styles.messagesContainer}>
            {messages.length === 0 && (
              <div className={styles.emptyState}>
                <p>👋 Hello! I'm your AI assistant for this book.</p>
                <p>Ask me questions like:</p>
                <ul>
                  <li onClick={() => setInputValue("What is ROS 2?")}>
                    "What is ROS 2?"
                  </li>
                  <li onClick={() => setInputValue("How do I set up NVIDIA Isaac Sim?")}>
                    "How do I set up NVIDIA Isaac Sim?"
                  </li>
                  <li onClick={() => setInputValue("Explain bipedal locomotion")}>
                    "Explain bipedal locomotion"
                  </li>
                </ul>
              </div>
            )}

            {messages.map((message) => (
              <div
                key={message.id}
                className={`${styles.message} ${styles[message.role]}`}
              >
                <div className={styles.messageContent}>
                  {message.role === 'assistant' ? (
                    <ReactMarkdown>{message.content}</ReactMarkdown>
                  ) : (
                    <p>{message.content}</p>
                  )}
                </div>
                {message.metadata && (
                  <div className={styles.messageMetadata}>
                    {message.metadata.latency_ms}ms · {message.metadata.tokens_used} tokens
                  </div>
                )}
              </div>
            ))}

            {isLoading && (
              <div className={`${styles.message} ${styles.assistant}`}>
                <div className={styles.loadingDots}>
                  <span></span>
                  <span></span>
                  <span></span>
                </div>
              </div>
            )}

            <div ref={messagesEndRef} />
          </div>

          {/* Input */}
          <div className={styles.inputContainer}>
            <input
              ref={inputRef}
              type="text"
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder="Ask a question..."
              disabled={isLoading}
              className={styles.input}
            />
            <button
              onClick={handleSendMessage}
              disabled={!inputValue.trim() || isLoading}
              className={styles.sendButton}
              aria-label="Send message"
            >
              <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                <path d="M22 2L11 13M22 2l-7 20-4-9-9-4 20-7z" />
              </svg>
            </button>
          </div>
        </div>
      )}
    </div>
  );
}
