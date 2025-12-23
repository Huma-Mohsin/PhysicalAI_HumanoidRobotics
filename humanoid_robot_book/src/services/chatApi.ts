/**
 * Chat API Service
 * Handles communication with the FastAPI backend for RAG chatbot
 */

// Use production backend by default
// For Docusaurus, use window object to access env vars if needed
const API_BASE_URL = typeof window !== 'undefined' && (window as any).ENV_API_URL
  ? (window as any).ENV_API_URL
  : 'https://humanoid-robotics-backend.vercel.app';  // Production backend

export interface ChatMessage {
  message_id: string;
  conversation_id: string;
  session_id: string;
  response: string;
  retrieved_chunks: RetrievedChunk[];
  metadata: MessageMetadata;
  timestamp: string;
}

export interface RetrievedChunk {
  chunk_id: string;
  chapter_id: string;
  chapter_title: string;
  section_title: string;
  content: string;
  similarity_score: number;
}

export interface MessageMetadata {
  latency_ms: number;
  tokens_used: number;
  model: string;
  qdrant_query_ms: number;
  retrieved_count: number;
  embedding_model: string;
}

export interface ChatQueryRequest {
  question: string;
  conversation_id?: string;
  session_id?: string;
  user_id?: string;  // Added for Better-Auth integration
  text_selection?: {
    text: string;
    chapter_id: string;
    start_offset: number;
    end_offset: number;
  };
  top_k?: number;
  similarity_threshold?: number;
}

export class ChatApiError extends Error {
  constructor(
    message: string,
    public statusCode?: number,
    public errorType?: string
  ) {
    super(message);
    this.name = 'ChatApiError';
  }
}

/**
 * Send a chat query to the backend
 */
export async function sendChatQuery(request: ChatQueryRequest): Promise<ChatMessage> {
  try {
    const response = await fetch(`${API_BASE_URL}/api/chat/query`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      credentials: 'include', // CRITICAL: Send cookies for authentication
      body: JSON.stringify(request),
    });

    if (!response.ok) {
      const errorData = await response.json().catch(() => ({}));
      throw new ChatApiError(
        errorData.detail?.message || `HTTP ${response.status}: ${response.statusText}`,
        response.status,
        errorData.detail?.error
      );
    }

    const data: ChatMessage = await response.json();
    return data;
  } catch (error) {
    if (error instanceof ChatApiError) {
      throw error;
    }

    // Network or other errors
    throw new ChatApiError(
      error instanceof Error ? error.message : 'Failed to send chat query',
      undefined,
      'NetworkError'
    );
  }
}

/**
 * Send a chat query with user authentication context
 */
export async function sendChatQueryWithAuth(
  question: string,
  user_id?: string,
  conversation_id?: string,
  session_id?: string,
  text_selection?: {
    text: string;
    chapter_id: string;
    start_offset: number;
    end_offset: number;
  }
): Promise<ChatMessage> {
  const request: ChatQueryRequest = {
    question,
    user_id,
    conversation_id,
    session_id,
    text_selection,
  };

  return sendChatQuery(request);
}

/**
 * Get session ID from localStorage or create a new one
 */
export function getOrCreateSessionId(): string {
  const storageKey = 'chatbot_session_id';
  let sessionId = localStorage.getItem(storageKey);

  if (!sessionId) {
    // Generate a simple UUID v4
    sessionId = 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, (c) => {
      const r = Math.random() * 16 | 0;
      const v = c === 'x' ? r : (r & 0x3 | 0x8);
      return v.toString(16);
    });
    localStorage.setItem(storageKey, sessionId);
  }

  return sessionId;
}

/**
 * Get conversation ID from localStorage
 */
export function getConversationId(): string | null {
  return localStorage.getItem('chatbot_conversation_id');
}

/**
 * Set conversation ID in localStorage
 */
export function setConversationId(conversationId: string): void {
  localStorage.setItem('chatbot_conversation_id', conversationId);
}

/**
 * Clear conversation (start new conversation)
 */
export function clearConversation(): void {
  localStorage.removeItem('chatbot_conversation_id');
}
