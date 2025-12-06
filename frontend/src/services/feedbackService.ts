/**
 * Feedback Service for chat message ratings
 */

export interface FeedbackRequest {
  message_id: string;
  rating: -1 | 0 | 1;  // -1: thumbs down, 0: neutral, 1: thumbs up
  comment?: string;
}

export interface FeedbackStats {
  total_feedback: number;
  positive_count: number;
  negative_count: number;
  positive_percentage: number;
  average_rating: number;
  recent_comments: string[];
}

const apiUrl = process.env.REACT_APP_API_URL || 'http://localhost:8000';

export class FeedbackService {
  /**
   * Submit feedback for a chat message
   */
  async submitFeedback(request: FeedbackRequest): Promise<void> {
    try {
      const response = await fetch(`${apiUrl}/feedback`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(request),
      });

      if (!response.ok) {
        const error = await response.json();
        throw new Error(error.detail || 'Failed to submit feedback');
      }
    } catch (error) {
      console.error('Feedback submission error:', error);
      throw error;
    }
  }

  /**
   * Get aggregated feedback statistics
   */
  async getStats(): Promise<FeedbackStats> {
    try {
      const response = await fetch(`${apiUrl}/feedback/stats`);

      if (!response.ok) {
        throw new Error('Failed to fetch feedback stats');
      }

      return await response.json();
    } catch (error) {
      console.error('Feedback stats error:', error);
      throw error;
    }
  }

  /**
   * Delete feedback for a message
   */
  async deleteFeedback(messageId: string): Promise<void> {
    try {
      const response = await fetch(`${apiUrl}/feedback/${messageId}`, {
        method: 'DELETE',
      });

      if (!response.ok) {
        throw new Error('Failed to delete feedback');
      }
    } catch (error) {
      console.error('Feedback deletion error:', error);
      throw error;
    }
  }
}

// Export singleton instance
export const feedbackService = new FeedbackService();
