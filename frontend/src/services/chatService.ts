/**
 * Chat Service for RAG API interactions
 */

export interface ChatMessage {
  role: 'user' | 'assistant';
  content: string;
}

export interface ChatRequest {
  query: string;
  language?: 'en' | 'ur';
  selected_text?: string;
  chat_history?: ChatMessage[];
  user_id?: string;
}

export interface RetrievedChunk {
  id: string;
  score: number;
  content: string;
  metadata: Record<string, any>;
}

export interface ChatResponse {
  response: string;
  retrieved_chunks: RetrievedChunk[];
  num_chunks: number;
  language: string;
  timestamp: string;
}

export class ChatService {
  private baseUrl: string;

  constructor() {
    // Use environment variable or default to localhost
    this.baseUrl = process.env.REACT_APP_API_URL || 'http://localhost:8000';
  }

  /**
   * Send a chat query to the RAG API
   */
  async sendQuery(request: ChatRequest): Promise<ChatResponse> {
    try {
      const response = await fetch(`${this.baseUrl}/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query: request.query,
          language: request.language || 'en',
          selected_text: request.selected_text,
          chat_history: request.chat_history,
          user_id: request.user_id,
        }),
      });

      if (!response.ok) {
        const error = await response.json();
        throw new Error(error.detail || 'Failed to get chat response');
      }

      return await response.json();
    } catch (error) {
      console.error('Chat service error:', error);
      throw error;
    }
  }

  /**
   * Check chat service health
   */
  async checkHealth(): Promise<{ status: string; rag_service: boolean }> {
    try {
      const response = await fetch(`${this.baseUrl}/chat/health`);
      if (!response.ok) {
        throw new Error('Health check failed');
      }
      return await response.json();
    } catch (error) {
      console.error('Health check error:', error);
      return { status: 'unavailable', rag_service: false };
    }
  }
}

// Export singleton instance
export const chatService = new ChatService();
