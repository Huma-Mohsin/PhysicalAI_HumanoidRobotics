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

  useEffect(() => {
    // Immediate selection handler without delay
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

              // Calculate position to appear immediately
              const buttonTop = rect.top + window.scrollY - 40; // Position above selection
              const buttonLeft = rect.left + rect.width / 2 + window.scrollX;

              setButtonPosition({
                top: buttonTop,
                left: buttonLeft,
              });

              // Show button immediately without delay
              setShowButton(true);
            }
          }
        } catch (e) {
          console.error('Error getting selection range:', e);
        }
      } else {
        setShowButton(false);
      }

      // Reset processing flag after a short delay
      setTimeout(() => {
        isProcessingSelection.current = false;
      }, 100);
    };

    const handleClickOutside = (e: MouseEvent) => {
      // Don't hide if clicking the button itself
      const target = e.target as HTMLElement;
      if (!target.closest('[data-text-selection-button]')) {
        setShowButton(false);
      }
    };

    // AGGRESSIVE context menu blocking - IMMEDIATE response
    const handleContextMenu = (e: MouseEvent | TouchEvent) => {
      const selection = window.getSelection();
      const text = selection?.toString().trim();

      // BLOCK IMMEDIATELY if there's any selected text
      if (text && text.length > 0) {
        e.preventDefault();
        e.stopPropagation();
        e.stopImmediatePropagation();

        // Handle selection immediately
        setTimeout(handleSelection, 0);

        return false;
      }
    };

    // Prevent context menu on content areas
    const handleContentContextMenu = (e: MouseEvent) => {
      const selection = window.getSelection();
      const text = selection?.toString().trim();

      if (text && text.length > 0) {
        e.preventDefault();
        e.stopPropagation();
        e.stopImmediatePropagation();
        return false;
      }
    };

    // Touch start handler to prevent context menu
    const handleTouchStart = (e: TouchEvent) => {
      const selection = window.getSelection();
      const text = selection?.toString().trim();

      if (text && text.length > 0) {
        e.preventDefault();
        e.stopPropagation();
        e.stopImmediatePropagation();
      }
    };

    // SELECTION CHANGE handler - MOST IMPORTANT for immediate response
    const handleSelectionChange = () => {
      // CRITICAL: Handle selection immediately without any delay
      const selection = window.getSelection();
      const text = selection?.toString().trim();

      if (text && text.length > 0) {
        // For very short text, we might want to wait a tiny bit to ensure it's a real selection
        if (text.length > 5) {
          handleSelection();
        } else {
          // For shorter text, still check but with a minimal delay to avoid flickering
          if (selectionTimeoutRef.current) {
            clearTimeout(selectionTimeoutRef.current);
          }
          selectionTimeoutRef.current = setTimeout(() => {
            handleSelection();
          }, 50); // Very minimal delay
        }
      } else {
        // If no text is selected, hide the button immediately
        setShowButton(false);
      }
    };

    // Prevent selection start that could lead to context menu
    const handleSelectStart = (e: Event) => {
      const selection = window.getSelection();
      const text = selection?.toString().trim();

      if (text && text.length > 0) {
        // Prevent default to avoid context menu, but handle our own selection
        e.preventDefault();
        e.stopPropagation();
        e.stopImmediatePropagation();

        // Process selection immediately
        setTimeout(handleSelection, 0);

        return false;
      }
    };

    // Mouse down handler for immediate response on desktop
    const handleMouseDown = (e: MouseEvent) => {
      // Check if there's already selected text when mouse is pressed
      const selection = window.getSelection();
      const text = selection?.toString().trim();

      if (text && text.length > 0) {
        // Prevent any context menu from appearing
        e.preventDefault();
        e.stopPropagation();
        e.stopImmediatePropagation();
      }
    };

    // Mouse up handler for immediate response on desktop
    const handleMouseUp = (e: MouseEvent) => {
      // Process selection immediately when mouse is released
      setTimeout(handleSelection, 0);
    };

    // Touch move handler to prevent context menu during selection
    const handleTouchMove = (e: TouchEvent) => {
      const selection = window.getSelection();
      const text = selection?.toString().trim();

      if (text && text.length > 0) {
        e.preventDefault();
        e.stopPropagation();
        e.stopImmediatePropagation();
      }
    };

    // Touch context menu handler
    const handleTouchContextMenu = (e: TouchEvent) => {
      const selection = window.getSelection();
      const text = selection?.toString().trim();

      if (text && text.length > 0) {
        e.preventDefault();
        e.stopPropagation();
        e.stopImmediatePropagation();
        return false;
      }
    };

    // Use selectionchange as the primary event for immediate response
    document.addEventListener('selectionchange', handleSelectionChange);

    // AGGRESSIVE desktop event handling for immediate response
    document.addEventListener('mousedown', handleMouseDown, true); // Capture phase
    document.addEventListener('mouseup', handleMouseUp, true); // Capture phase

    // Additional events for comprehensive coverage
    document.addEventListener('click', handleClickOutside);

    // AGGRESSIVE context menu blocking
    document.addEventListener('contextmenu', handleContextMenu as any, true); // Capture
    document.addEventListener('contextmenu', handleContentContextMenu, true); // Capture
    document.addEventListener('contextmenu', handleContextMenu as any, false); // Bubble

    // Selection and touch event blocking
    document.addEventListener('selectstart', handleSelectStart as any);
    document.addEventListener('touchstart', handleTouchStart as any, { passive: false });
    document.addEventListener('touchmove', handleTouchMove as any, { passive: false });
    document.addEventListener('touchcancel', handleTouchContextMenu as any, true);

    // Apply to main content areas as well
    const mainContent = document.querySelector('main, article, [role="main"], .markdown, .theme-doc-markdown');
    if (mainContent) {
      mainContent.addEventListener('contextmenu', handleContentContextMenu, true);
      mainContent.addEventListener('touchstart', handleTouchStart as any, { passive: false });
      mainContent.addEventListener('selectstart', handleSelectStart as any);
      mainContent.addEventListener('touchmove', handleTouchMove as any, { passive: false });
      mainContent.addEventListener('touchcancel', handleTouchContextMenu as any, true);
      mainContent.addEventListener('selectionchange', handleSelectionChange);
      mainContent.addEventListener('mousedown', handleMouseDown, true);
      mainContent.addEventListener('mouseup', handleMouseUp, true);
    }

    return () => {
      // Clear timeout on cleanup
      if (selectionTimeoutRef.current) {
        clearTimeout(selectionTimeoutRef.current);
      }

      // Remove all event listeners
      document.removeEventListener('selectionchange', handleSelectionChange);
      document.removeEventListener('mousedown', handleMouseDown, true);
      document.removeEventListener('mouseup', handleMouseUp, true);
      document.removeEventListener('click', handleClickOutside);
      document.removeEventListener('contextmenu', handleContextMenu as any, true);
      document.removeEventListener('contextmenu', handleContentContextMenu, true);
      document.removeEventListener('contextmenu', handleContextMenu as any, false);
      document.removeEventListener('selectstart', handleSelectStart as any);
      document.removeEventListener('touchstart', handleTouchStart as any);
      document.removeEventListener('touchmove', handleTouchMove as any);
      document.removeEventListener('touchcancel', handleTouchContextMenu as any);

      if (mainContent) {
        mainContent.removeEventListener('contextmenu', handleContentContextMenu, true);
        mainContent.removeEventListener('touchstart', handleTouchStart as any);
        mainContent.removeEventListener('selectstart', handleSelectStart as any);
        mainContent.removeEventListener('touchmove', handleTouchMove as any);
        mainContent.removeEventListener('touchcancel', handleTouchContextMenu as any);
        mainContent.removeEventListener('selectionchange', handleSelectionChange);
        mainContent.removeEventListener('mousedown', handleMouseDown, true);
        mainContent.removeEventListener('mouseup', handleMouseUp, true);
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
