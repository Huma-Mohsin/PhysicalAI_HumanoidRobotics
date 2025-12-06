/**
 * Analytics Dashboard
 *
 * Displays platform usage statistics and feedback metrics
 */

import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import { feedbackService, FeedbackStats } from '../services/feedbackService';
import styles from './analytics.module.css';

export default function Analytics() {
  const [stats, setStats] = useState<FeedbackStats | null>(null);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    loadStats();
  }, []);

  const loadStats = async () => {
    try {
      setIsLoading(true);
      const data = await feedbackService.getStats();
      setStats(data);
    } catch (err) {
      setError('Failed to load statistics');
      console.error(err);
    } finally {
      setIsLoading(false);
    }
  };

  if (isLoading) {
    return (
      <Layout title="Analytics" description="Platform analytics and feedback metrics">
        <div className={styles.container}>
          <div className={styles.loading}>Loading statistics...</div>
        </div>
      </Layout>
    );
  }

  if (error) {
    return (
      <Layout title="Analytics" description="Platform analytics and feedback metrics">
        <div className={styles.container}>
          <div className={styles.error}>{error}</div>
        </div>
      </Layout>
    );
  }

  return (
    <Layout title="Analytics" description="Platform analytics and feedback metrics">
      <div className={styles.container}>
        <h1>📊 Platform Analytics</h1>
        <p className={styles.subtitle}>Usage statistics and feedback insights</p>

        <div className={styles.grid}>
          {/* Total Feedback */}
          <div className={styles.card}>
            <div className={styles.cardIcon}>💬</div>
            <div className={styles.cardContent}>
              <div className={styles.cardValue}>{stats?.total_feedback || 0}</div>
              <div className={styles.cardLabel}>Total Feedback</div>
            </div>
          </div>

          {/* Positive Feedback */}
          <div className={`${styles.card} ${styles.positive}`}>
            <div className={styles.cardIcon}>👍</div>
            <div className={styles.cardContent}>
              <div className={styles.cardValue}>{stats?.positive_count || 0}</div>
              <div className={styles.cardLabel}>Positive Ratings</div>
            </div>
          </div>

          {/* Negative Feedback */}
          <div className={`${styles.card} ${styles.negative}`}>
            <div className={styles.cardIcon}>👎</div>
            <div className={styles.cardContent}>
              <div className={styles.cardValue}>{stats?.negative_count || 0}</div>
              <div className={styles.cardLabel}>Negative Ratings</div>
            </div>
          </div>

          {/* Satisfaction Rate */}
          <div className={`${styles.card} ${styles.highlight}`}>
            <div className={styles.cardIcon}>⭐</div>
            <div className={styles.cardContent}>
              <div className={styles.cardValue}>
                {stats?.positive_percentage.toFixed(1) || 0}%
              </div>
              <div className={styles.cardLabel}>Satisfaction Rate</div>
            </div>
          </div>
        </div>

        {/* Progress Bar */}
        {stats && stats.total_feedback > 0 && (
          <div className={styles.progressSection}>
            <h2>Feedback Distribution</h2>
            <div className={styles.progressBar}>
              <div
                className={styles.progressPositive}
                style={{ width: `${stats.positive_percentage}%` }}
              >
                {stats.positive_percentage > 10 && `${stats.positive_percentage.toFixed(0)}%`}
              </div>
              <div
                className={styles.progressNegative}
                style={{
                  width: `${100 - stats.positive_percentage}%`,
                }}
              >
                {100 - stats.positive_percentage > 10 &&
                  `${(100 - stats.positive_percentage).toFixed(0)}%`}
              </div>
            </div>
            <div className={styles.progressLabels}>
              <span>👍 Positive</span>
              <span>👎 Negative</span>
            </div>
          </div>
        )}

        {/* Recent Comments */}
        {stats && stats.recent_comments.length > 0 && (
          <div className={styles.commentsSection}>
            <h2>Recent Feedback Comments</h2>
            <div className={styles.commentsList}>
              {stats.recent_comments.map((comment, index) => (
                <div key={index} className={styles.commentCard}>
                  <div className={styles.commentIcon}>💬</div>
                  <div className={styles.commentText}>{comment}</div>
                </div>
              ))}
            </div>
          </div>
        )}

        {/* Empty State */}
        {stats && stats.total_feedback === 0 && (
          <div className={styles.emptyState}>
            <div className={styles.emptyIcon}>📭</div>
            <h3>No feedback yet</h3>
            <p>Start using the AI chatbot and provide feedback to see analytics here!</p>
          </div>
        )}
      </div>
    </Layout>
  );
}
