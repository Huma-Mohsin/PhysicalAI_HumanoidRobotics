/**
 * Text Selection Component
 * Allows users to select text and ask questions about it
 * SAFE: Non-breaking, optional feature
 */

import React, { useEffect, useState, useRef } from 'react';
import styles from './TextSelection.module.css';

interface TextSelectionProps {
  onAskAboutSelection: (selectedText: string) => void;
}

export default function TextSelection({ onAskAboutSelection }: TextSelectionProps): JSX.Element {
  const [selectedText, setSelectedText] = useState('');
  const [buttonPosition, setButtonPosition] = useState({ top: 0, left: 0 });
  const [showButton, setShowButton] = useState(false);
  const selectionTimeoutRef = useRef<NodeJS.Timeout | null>(null);
  const isProcessingSelection = useRef(false); // Prevent duplicate processing
  const lastSelection = useRef<string>(''); // Track last selection to detect changes

  useEffect(() => {
    // Selection handler with mobile-first approach
    const handleSelection = () => {
      // Prevent duplicate processing
      if (isProcessingSelection.current) return;

      isProcessingSelection.current = true;

      // Clear any existing timeout
      if (selectionTimeoutRef.current) {
        clearTimeout(selectionTimeoutRef.current);
        selectionTimeoutRef.current = null;
      }

      const selection = window.getSelection();
      const text = selection?.toString().trim();

      if (text && text.length > 5) {
        try {
          const range = selection?.getRangeAt(0);
          if (range) {
            const rect = range.getBoundingClientRect();

            if (rect && rect.width > 0 && rect.height > 0) {
              setSelectedText(text);
              lastSelection.current = text; // Track this selection

              // Calculate position to appear immediately with no delay
              // Use viewport-relative coordinates for mobile compatibility
              const buttonTop = rect.top + window.pageYOffset - 40; // Position above selection
              const buttonLeft = rect.left + window.pageXOffset + rect.width / 2;

              setButtonPosition({
                top: buttonTop,
                left: buttonLeft,
              });

              // Show button immediately with no delay
              setShowButton(true);
            }
          }
        } catch (e) {
          console.error('Error getting selection range:', e);
        }
      } else {
        // If no text is selected, hide the button immediately
        setShowButton(false);
        lastSelection.current = ''; // Clear last selection
      }

      // Reset processing flag quickly to allow new selections
      setTimeout(() => {
        isProcessingSelection.current = false;
      }, 100); // Slightly longer timeout to ensure mobile stability
    };

    // Touch end handler for immediate response on mobile
    const handleTouchEnd = (e: TouchEvent) => {
      // Process selection immediately when touch ends
      setTimeout(() => {
        handleSelection();
      }, 10); // Slightly delayed to allow other touch events to process
    };

    // Mouse up handler for immediate response on desktop
    const handleMouseUp = (e: MouseEvent) => {
      // Process selection immediately when mouse is released
      setTimeout(() => {
        handleSelection();
      }, 10); // Slightly delayed to allow other events to process
    };

    // SELECTION CHANGE handler - PRIMARY for immediate response
    const handleSelectionChange = () => {
      // Prevent duplicate processing
      if (isProcessingSelection.current) return;

      // CRITICAL: Handle selection immediately without any delay
      const selection = window.getSelection();
      const text = selection?.toString().trim();

      if (text && text.length > 5) { // Increased threshold to avoid showing button for small selections
        // For immediate response, handle selection right away
        handleSelection();
      } else {
        // If no text is selected, hide the button immediately
        setShowButton(false);
        lastSelection.current = ''; // Clear last selection
      }
    };

    // Mobile-optimized context menu handler
    const handleContextMenu = (e: MouseEvent | TouchEvent) => {
      const selection = window.getSelection();
      const text = selection?.toString().trim();

      // BLOCK ONLY if there's selected text (this prevents the mini menu)
      // But be minimal to not interfere with other mobile gestures
      if (text && text.length > 5) {
        e.preventDefault();
      }
    };

    // Use selectionchange as the primary event for immediate response
    document.addEventListener('selectionchange', handleSelectionChange);

    // Desktop event handling
    document.addEventListener('mouseup', handleMouseUp, true); // Capture phase

    // Mobile event handling - MINIMAL approach
    document.addEventListener('touchend', handleTouchEnd, { passive: true }); // Use passive for better mobile performance
    document.addEventListener('contextmenu', handleContextMenu as any, true); // Capture

    // Apply to main content areas as well with minimal event listeners
    const mainContent = document.querySelector('main, article, [role="main"], .markdown, .theme-doc-markdown');
    if (mainContent) {
      mainContent.addEventListener('selectionchange', handleSelectionChange);
      mainContent.addEventListener('mouseup', handleMouseUp, true);
      mainContent.addEventListener('touchend', handleTouchEnd, { passive: true });
      mainContent.addEventListener('contextmenu', handleContextMenu as any, true);
    }

    return () => {
      // Clear timeout on cleanup
      if (selectionTimeoutRef.current) {
        clearTimeout(selectionTimeoutRef.current);
      }

      // Remove all event listeners - use same options as when adding
      document.removeEventListener('selectionchange', handleSelectionChange);
      document.removeEventListener('mouseup', handleMouseUp, true);
      document.removeEventListener('touchend', handleTouchEnd, { passive: true });
      document.removeEventListener('contextmenu', handleContextMenu as any, true);

      if (mainContent) {
        mainContent.removeEventListener('selectionchange', handleSelectionChange);
        mainContent.removeEventListener('mouseup', handleMouseUp, true);
        mainContent.removeEventListener('touchend', handleTouchEnd, { passive: true });
        mainContent.removeEventListener('contextmenu', handleContextMenu as any, true);
      }
    };
  }, []);

  const handleAskClick = (e: React.MouseEvent) => {
    // Minimal event handling to preserve mobile functionality
    e.stopPropagation();
    if (selectedText) {
      // Temporarily disable selection processing to avoid conflicts
      isProcessingSelection.current = true;

      // Clear selection to avoid any lingering selection state
      const selection = window.getSelection();
      if (selection) {
        selection.removeAllRanges();
      }

      // Hide the button immediately
      setShowButton(false);
      setSelectedText(''); // Clear the selected text state
      lastSelection.current = ''; // Clear the last selection reference

      // Reset processing flag quickly to allow other events to process
      setTimeout(() => {
        isProcessingSelection.current = false;
      }, 100); // Slightly longer timeout for mobile stability

      // Call the parent callback to handle the question after a brief delay
      // to ensure UI updates complete first
      setTimeout(() => {
        onAskAboutSelection(selectedText);
      }, 50); // Slightly longer delay to ensure proper cleanup
    }
  };

  // Also hide the button when clicking anywhere if selection is cleared
  useEffect(() => {
    const handleGlobalClick = (e: MouseEvent) => {
      // Don't hide if clicking the button itself
      const target = e.target as HTMLElement;
      if (target.closest('[data-text-selection-button]')) {
        return;
      }

      const selection = window.getSelection();
      const currentText = selection?.toString().trim();
      if (!currentText || currentText.length === 0) {
        setShowButton(false);
      }
    };

    document.addEventListener('click', handleGlobalClick);
    return () => {
      document.removeEventListener('click', handleGlobalClick);
    };
  }, []);

  if (!showButton) return null;

  return (
    <button
      className={`${styles.askButton} ${styles.show}`}
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
