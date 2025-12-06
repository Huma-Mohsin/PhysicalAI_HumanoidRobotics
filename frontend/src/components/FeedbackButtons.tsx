/**
 * Feedback Buttons Component
 *
 * Thumbs up/down buttons for chat messages
 */

import React, { useState } from 'react';
import { feedbackService } from '../services/feedbackService';
import styles from './FeedbackButtons.module.css';

interface FeedbackButtonsProps {
  messageId: string;
  onFeedbackSubmitted?: () => void;
}

export const FeedbackButtons: React.FC<FeedbackButtonsProps> = ({
  messageId,
  onFeedbackSubmitted,
}) => {
  const [selectedRating, setSelectedRating] = useState<-1 | 0 | 1 | null>(null);
  const [isSubmitting, setIsSubmitting] = useState(false);
  const [showCommentBox, setShowCommentBox] = useState(false);
  const [comment, setComment] = useState('');

  const handleRating = async (rating: -1 | 1) => {
    if (isSubmitting) return;

    setIsSubmitting(true);

    try {
      await feedbackService.submitFeedback({
        message_id: messageId,
        rating,
        comment: comment || undefined,
      });

      setSelectedRating(rating);
      setShowCommentBox(false);
      setComment('');

      if (onFeedbackSubmitted) {
        onFeedbackSubmitted();
      }
    } catch (error) {
      console.error('Failed to submit feedback:', error);
    } finally {
      setIsSubmitting(false);
    }
  };

  const handleCommentSubmit = () => {
    if (selectedRating !== null) {
      handleRating(selectedRating);
    }
  };

  return (
    <div className={styles.feedbackContainer}>
      <div className={styles.buttonGroup}>
        <button
          className={`${styles.feedbackBtn} ${
            selectedRating === 1 ? styles.selected : ''
          }`}
          onClick={() => handleRating(1)}
          disabled={isSubmitting}
          title="Helpful response"
          aria-label="Thumbs up"
        >
          👍
        </button>

        <button
          className={`${styles.feedbackBtn} ${
            selectedRating === -1 ? styles.selected : ''
          }`}
          onClick={() => {
            if (selectedRating === -1) {
              handleRating(-1);
            } else {
              setSelectedRating(-1);
              setShowCommentBox(true);
            }
          }}
          disabled={isSubmitting}
          title="Not helpful"
          aria-label="Thumbs down"
        >
          👎
        </button>

        {selectedRating !== null && !showCommentBox && (
          <button
            className={styles.commentBtn}
            onClick={() => setShowCommentBox(true)}
            title="Add comment"
          >
            💬
          </button>
        )}
      </div>

      {showCommentBox && (
        <div className={styles.commentBox}>
          <textarea
            value={comment}
            onChange={(e) => setComment(e.target.value)}
            placeholder="Tell us more... (optional)"
            className={styles.commentInput}
            maxLength={500}
            rows={3}
          />
          <div className={styles.commentActions}>
            <button
              onClick={handleCommentSubmit}
              className={styles.submitBtn}
              disabled={isSubmitting}
            >
              {isSubmitting ? 'Submitting...' : 'Submit'}
            </button>
            <button
              onClick={() => {
                setShowCommentBox(false);
                setComment('');
              }}
              className={styles.cancelBtn}
            >
              Cancel
            </button>
          </div>
        </div>
      )}

      {selectedRating !== null && !showCommentBox && (
        <span className={styles.thankYouMsg}>Thanks for your feedback!</span>
      )}
    </div>
  );
};
