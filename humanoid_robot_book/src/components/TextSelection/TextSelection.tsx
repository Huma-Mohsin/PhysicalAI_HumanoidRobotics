/**
 * Text Selection Component
 * Allows users to select text and ask questions about it
 * SAFE: Non-breaking, optional feature
 */

import React, { useEffect, useState } from 'react';
import styles from './TextSelection.module.css';

interface TextSelectionProps {
  onAskAboutSelection: (selectedText: string) => void;
}

export default function TextSelection({ onAskAboutSelection }: TextSelectionProps): JSX.Element {
  const [selectedText, setSelectedText] = useState('');
  const [buttonPosition, setButtonPosition] = useState({ top: 0, left: 0 });
  const [showButton, setShowButton] = useState(false);

  useEffect(() => {
    const handleSelection = () => {
      // Longer delay for mobile to ensure selection is properly captured
      setTimeout(() => {
        const selection = window.getSelection();
        const text = selection?.toString().trim();

        if (text && text.length > 5) {
          // Only show if substantial text selected
          try {
            const range = selection?.getRangeAt(0);
            const rect = range?.getBoundingClientRect();

            if (rect && rect.width > 0 && rect.height > 0) {
              setSelectedText(text);
              setButtonPosition({
                top: rect.top + window.scrollY - 50,
                left: rect.left + rect.width / 2,
              });
              setShowButton(true);
            }
          } catch (e) {
            console.error('Error getting selection range:', e);
          }
        } else {
          setShowButton(false);
        }
      }, 150); // Increased delay for mobile touch events
    };

    const handleClickOutside = (e: MouseEvent) => {
      // Don't hide if clicking the button itself
      const target = e.target as HTMLElement;
      if (!target.closest('[data-text-selection-button]')) {
        setShowButton(false);
      }
    };

    // Disable default context menu on text selection - AGGRESSIVE BLOCKING (Desktop + Mobile)
    const handleContextMenu = (e: MouseEvent | TouchEvent) => {
      const selection = window.getSelection();
      const text = selection?.toString().trim();

      // If there's ANY selected text, COMPLETELY BLOCK default context menu
      if (text && text.length > 0) {
        e.preventDefault();
        e.stopPropagation();
        e.stopImmediatePropagation();

        // Force return false for older browsers
        if (e.cancelable) {
          e.preventDefault();
        }

        return false;
      }
    };

    // Additional mobile-specific long-press blocking
    const handleTouchStart = (e: TouchEvent) => {
      // Prevent mobile long-press context menu
      const target = e.target as HTMLElement;
      if (target.closest('.markdown, article, .theme-doc-markdown, main')) {
        // Allow text selection but prevent context menu
        target.style.webkitUserSelect = 'text';
        target.style.userSelect = 'text';
        target.style.webkitTouchCallout = 'none';
      }
    };

    // Additional blocking for specific content areas
    const handleContentContextMenu = (e: MouseEvent) => {
      const target = e.target as HTMLElement;
      // Block context menu in main content and docs areas
      if (target.closest('.markdown, article, .theme-doc-markdown, main')) {
        const selection = window.getSelection();
        if (selection && selection.toString().trim().length > 0) {
          e.preventDefault();
          e.stopPropagation();
          e.stopImmediatePropagation();
          return false;
        }
      }
    };

    // Use mouseup for desktop and touchend for mobile - selectionchange fires too early and causes issues
    document.addEventListener('mouseup', handleSelection);
    document.addEventListener('touchend', handleSelection); // Mobile touch support
    document.addEventListener('click', handleClickOutside);

    // Mobile: Prevent long-press context menu at touchstart
    document.addEventListener('touchstart', handleTouchStart, { passive: false });

    // MULTIPLE LAYERS of context menu blocking - capture phase + bubble phase
    document.addEventListener('contextmenu', handleContextMenu as any, true); // Capture
    document.addEventListener('contextmenu', handleContentContextMenu, true); // Capture
    document.addEventListener('contextmenu', handleContextMenu as any, false); // Bubble

    // Also block on main content element if it exists
    const mainContent = document.querySelector('main, article, [role="main"]');
    if (mainContent) {
      mainContent.addEventListener('contextmenu', handleContentContextMenu, true);
      mainContent.addEventListener('touchstart', handleTouchStart as any, { passive: false });
    }

    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('touchend', handleSelection); // Mobile touch cleanup
      document.removeEventListener('click', handleClickOutside);
      document.removeEventListener('touchstart', handleTouchStart as any);
      document.removeEventListener('contextmenu', handleContextMenu as any, true);
      document.removeEventListener('contextmenu', handleContentContextMenu, true);
      document.removeEventListener('contextmenu', handleContextMenu as any, false);

      if (mainContent) {
        mainContent.removeEventListener('contextmenu', handleContentContextMenu, true);
        mainContent.removeEventListener('touchstart', handleTouchStart as any);
      }
    };
  }, []);

  const handleAskClick = (e: React.MouseEvent) => {
    e.stopPropagation();
    if (selectedText) {
      onAskAboutSelection(selectedText);
      setShowButton(false);
    }
  };

  if (!showButton) return null;

  return (
    <button
      className={styles.askButton}
      style={{
        position: 'fixed',
        top: `${buttonPosition.top}px`,
        left: `${buttonPosition.left}px`,
      }}
      onClick={handleAskClick}
      title="Ask chatbot about this selection"
      data-text-selection-button="true"
    >
      <svg
        width="18"
        height="18"
        viewBox="0 0 24 24"
        fill="none"
        stroke="currentColor"
        strokeWidth="2.5"
        strokeLinecap="round"
        strokeLinejoin="round"
      >
        <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" />
        <circle cx="9" cy="10" r="0.5" fill="white" />
        <circle cx="12" cy="10" r="0.5" fill="white" />
        <circle cx="15" cy="10" r="0.5" fill="white" />
      </svg>
      💬 Ask Me
    </button>
  );
}
