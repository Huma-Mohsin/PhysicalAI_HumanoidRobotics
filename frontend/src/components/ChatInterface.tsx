/**
 * Chat Interface Component for RAG Queries
 *
 * Features:
 * - Real-time AI responses using RAG pipeline
 * - Chat history with user/assistant messages
 * - Context-scoped queries (selected text support)
 * - Mobile-responsive design
 * - Loading states and error handling
 */

import React, { useState, useRef, useEffect } from 'react';
import { chatService, ChatMessage, ChatResponse } from '../services/chatService';
import { useLanguage } from '../contexts/LanguageContext';
import { useAuth } from '../hooks/useAuth';
import './ChatInterface.css';

interface ChatInterfaceProps {
  selectedText?: string;
}

export const ChatInterface: React.FC<ChatInterfaceProps> = ({
  selectedText
}) => {
  const { language } = useLanguage();
  const { user } = useAuth();
  const userId = user?.id;
  const [messages, setMessages] = useState<ChatMessage[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [isExpanded, setIsExpanded] = useState(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Auto-scroll to latest message
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  // Show context indicator if text is selected
  useEffect(() => {
    if (selectedText && !isExpanded) {
      setIsExpanded(true);
    }
  }, [selectedText]);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();

    if (!inputValue.trim() || isLoading) return;

    const userMessage: ChatMessage = {
      role: 'user',
      content: inputValue.trim()
    };

    // Add user message to chat
    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setError(null);
    setIsLoading(true);

    try {
      // Call RAG API
      const response: ChatResponse = await chatService.sendQuery({
        query: userMessage.content,
        language,
        selected_text: selectedText,
        chat_history: messages,
        user_id: userId
      });

      // Add assistant response
      const assistantMessage: ChatMessage = {
        role: 'assistant',
        content: response.response
      };

      setMessages(prev => [...prev, assistantMessage]);
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to get response');
      console.error('Chat error:', err);
    } finally {
      setIsLoading(false);
    }
  };

  const clearChat = () => {
    setMessages([]);
    setError(null);
  };

  return (
    <div className={`chat-interface ${isExpanded ? 'expanded' : 'collapsed'}`}>
      {/* Toggle Button */}
      <button
        className="chat-toggle"
        onClick={() => setIsExpanded(!isExpanded)}
        aria-label={isExpanded ? 'Minimize chat' : 'Expand chat'}
      >
        {isExpanded ? '−' : '💬 Ask AI'}
      </button>

      {isExpanded && (
        <div className="chat-container">
          {/* Header */}
          <div className="chat-header">
            <h3>🤖 Physical AI Assistant</h3>
            {messages.length > 0 && (
              <button onClick={clearChat} className="clear-btn">
                Clear
              </button>
            )}
          </div>

          {/* Context Indicator */}
          {selectedText && (
            <div className="context-indicator">
              📌 Asking about selected text
            </div>
          )}

          {/* Messages */}
          <div className="messages-container">
            {messages.length === 0 && (
              <div className="empty-state">
                <p>👋 Ask me anything about Physical AI, ROS 2, robotics, or simulation!</p>
                <p className="hint">Try: "How do I create a ROS 2 node with GPT-4?"</p>
              </div>
            )}

            {messages.map((msg, index) => (
              <div
                key={index}
                className={`message ${msg.role}`}
              >
                <div className="message-avatar">
                  {msg.role === 'user' ? '👤' : '🤖'}
                </div>
                <div className="message-content">
                  {msg.content}
                </div>
              </div>
            ))}

            {isLoading && (
              <div className="message assistant loading">
                <div className="message-avatar">🤖</div>
                <div className="message-content">
                  <div className="typing-indicator">
                    <span></span>
                    <span></span>
                    <span></span>
                  </div>
                </div>
              </div>
            )}

            {error && (
              <div className="error-message">
                ⚠️ {error}
              </div>
            )}

            <div ref={messagesEndRef} />
          </div>

          {/* Input Form */}
          <form onSubmit={handleSubmit} className="chat-input-form">
            <input
              type="text"
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              placeholder="Ask a question..."
              disabled={isLoading}
              className="chat-input"
            />
            <button
              type="submit"
              disabled={isLoading || !inputValue.trim()}
              className="send-btn"
            >
              {isLoading ? '⏳' : '➤'}
            </button>
          </form>
        </div>
      )}
    </div>
  );
};
