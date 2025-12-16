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
  const touchStartTimeRef = useRef<number>(0); // Track touch start time for mobile

  useEffect(() => {
    // Disable browser's default text selection mini menu and context menus
    const disableTextSelectionMenu = () => {
      // Add styles to disable text selection menu
      const style = document.createElement('style');
      style.textContent = `
        * {
          -webkit-touch-callout: none !important;
          -webkit-user-select: text !important;
          -khtml-user-select: text !important;
          -moz-user-select: text !important;
          -ms-user-select: text !important;
          user-select: text !important;
        }

        /* Disable the default text selection context menu */
        ::selection {
          background: auto;
        }

        /* Prevent the default selection toolbar on mobile */
        input[type="text"], textarea, *[contenteditable="true"] {
          -webkit-touch-callout: default;
          -webkit-user-select: auto;
          user-select: auto;
        }

        /* More aggressive mobile-specific styling */
        .markdown, article, .theme-doc-markdown, main, p, div, span, h1, h2, h3, h4, h5, h6, li, td, th {
          -webkit-touch-callout: none !important;
          -webkit-user-select: text !important;
          -moz-user-select: text !important;
          -ms-user-select: text !important;
          user-select: text !important;
        }
      `;
      document.head.appendChild(style);

      // Return cleanup function
      return () => {
        document.head.removeChild(style);
      };
    };

    // Call the function to disable default text selection menu
    const cleanupStyle = disableTextSelectionMenu();

    // Immediate selection handler without delay - MOBILE OPTIMIZED
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
      }, 10); // Even faster timeout for mobile
    };

    const handleClickOutside = (e: MouseEvent) => {
      // Don't hide if clicking the button itself
      const target = e.target as HTMLElement;
      if (!target.closest('[data-text-selection-button]')) {
        // Hide button when clicking outside, but only if no new selection is made
        const currentSelection = window.getSelection()?.toString().trim();
        if (!currentSelection || currentSelection.length === 0) {
          setShowButton(false);
        }
      }
    };

    // AGGRESSIVE context menu blocking - IMMEDIATE response on MOBILE
    const handleContextMenu = (e: MouseEvent | TouchEvent) => {
      const selection = window.getSelection();
      const text = selection?.toString().trim();

      // BLOCK IMMEDIATELY if there's any selected text - MORE AGGRESSIVE for mobile
      if (text && text.length > 0) {
        e.preventDefault();
        e.stopPropagation();
        e.stopImmediatePropagation();

        // Handle selection immediately
        setTimeout(() => {
          handleSelection();
        }, 1); // Minimal delay to ensure event processing

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

    // MORE AGGRESSIVE touch start handler to prevent context menu on mobile
    const handleTouchStart = (e: TouchEvent) => {
      touchStartTimeRef.current = Date.now(); // Track touch start time

      // Prevent default behavior for text selection areas to stop context menu
      const target = e.target as HTMLElement;
      if (target.closest('.markdown, article, .theme-doc-markdown, main, p, div, span, h1, h2, h3, h4, h5, h6, li, td, th')) {
        const selection = window.getSelection();
        const currentText = selection?.toString().trim();

        // If there's already selected text, we're in a selection operation - prevent default
        if (currentText && currentText.length > 0) {
          e.preventDefault();
          e.stopPropagation();
          e.stopImmediatePropagation();
        }
      }
    };

    // SELECTION CHANGE handler - MOST IMPORTANT for immediate response
    const handleSelectionChange = () => {
      // CRITICAL: Handle selection immediately without any delay
      const selection = window.getSelection();
      const text = selection?.toString().trim();

      if (text && text.length > 0) {
        // For immediate response on mobile, handle selection right away
        handleSelection();
      } else {
        // If no text is selected, hide the button immediately
        setShowButton(false);
        lastSelection.current = ''; // Clear last selection
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
        setTimeout(() => {
          handleSelection();
        }, 1); // Minimal delay

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
      setTimeout(() => {
        handleSelection();
      }, 1); // Minimal delay
    };

    // Touch end handler for immediate response on mobile
    const handleTouchEnd = (e: TouchEvent) => {
      // Calculate touch duration to determine if it was a selection vs tap
      const touchDuration = Date.now() - touchStartTimeRef.current;

      // Process selection immediately when touch ends
      setTimeout(() => {
        handleSelection();
      }, 1); // Minimal delay
    };

    // Touch force change handler for immediate response on mobile (for 3D Touch devices)
    const handleTouchForceChange = (e: TouchEvent) => {
      // Process selection immediately when touch force changes
      setTimeout(() => {
        handleSelection();
      }, 1); // Minimal delay
    };

    // Click handler to detect when user clicks elsewhere to deselect
    const handleClick = (e: MouseEvent) => {
      // Check if the click is outside of selected text and the ask button
      const target = e.target as HTMLElement;
      const selection = window.getSelection();
      const currentText = selection?.toString().trim();

      // If there's no selection or the selection is empty, hide the button
      if (!currentText || currentText.length === 0) {
        setShowButton(false);
      }

      // If user clicks somewhere that's not the ask button or selected text, hide the button
      if (!target.closest('[data-text-selection-button]') &&
          !target.closest('a, button, input, textarea')) { // Don't hide if clicking on interactive elements
        // But only hide if there's no new selection
        if (!currentText || currentText.length === 0) {
          setShowButton(false);
        }
      }
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

    // Touch context menu handler - MORE AGGRESSIVE for mobile
    const handleTouchContextMenu = (e: TouchEvent) => {
      const selection = window.getSelection();
      const text = selection?.toString().trim();

      if (text && text.length > 0) {
        e.preventDefault();
        e.stopPropagation();
        e.stopImmediatePropagation();
        // Process selection immediately to show Ask Me button
        setTimeout(() => {
          handleSelection();
        }, 1); // Minimal delay
        return false;
      }
    };

    // Use selectionchange as the primary event for immediate response
    document.addEventListener('selectionchange', handleSelectionChange);

    // AGGRESSIVE desktop event handling for immediate response
    document.addEventListener('mousedown', handleMouseDown, true); // Capture phase
    document.addEventListener('mouseup', handleMouseUp, true); // Capture phase

    // AGGRESSIVE mobile event handling for immediate response
    document.addEventListener('touchstart', handleTouchStart, { passive: false });
    document.addEventListener('touchend', handleTouchEnd, { passive: false });
    document.addEventListener('touchmove', handleTouchMove, { passive: false });
    document.addEventListener('touchcancel', handleTouchContextMenu, true);
    document.addEventListener('touchforcechange', handleTouchForceChange as any, { passive: false }); // For 3D Touch devices

    // Additional events for comprehensive coverage
    document.addEventListener('click', handleClick);
    document.addEventListener('click', handleClickOutside);

    // AGGRESSIVE context menu blocking
    document.addEventListener('contextmenu', handleContextMenu as any, true); // Capture
    document.addEventListener('contextmenu', handleContentContextMenu, true); // Capture
    document.addEventListener('contextmenu', handleContextMenu as any, false); // Bubble

    // Selection and touch event blocking
    document.addEventListener('selectstart', handleSelectStart as any);

    // Apply to main content areas as well
    const mainContent = document.querySelector('main, article, [role="main"], .markdown, .theme-doc-markdown');
    if (mainContent) {
      mainContent.addEventListener('contextmenu', handleContentContextMenu, true);
      mainContent.addEventListener('touchstart', handleTouchStart, { passive: false });
      mainContent.addEventListener('touchend', handleTouchEnd, { passive: false });
      mainContent.addEventListener('touchmove', handleTouchMove, { passive: false });
      mainContent.addEventListener('touchcancel', handleTouchContextMenu, true);
      mainContent.addEventListener('touchforcechange', handleTouchForceChange as any, { passive: false });
      mainContent.addEventListener('selectstart', handleSelectStart);
      mainContent.addEventListener('selectionchange', handleSelectionChange);
      mainContent.addEventListener('mousedown', handleMouseDown, true);
      mainContent.addEventListener('mouseup', handleMouseUp, true);
      mainContent.addEventListener('click', handleClick);
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
      document.removeEventListener('touchstart', handleTouchStart);
      document.removeEventListener('touchend', handleTouchEnd);
      document.removeEventListener('touchmove', handleTouchMove);
      document.removeEventListener('touchcancel', handleTouchContextMenu);
      document.removeEventListener('touchforcechange', handleTouchForceChange as any);
      document.removeEventListener('click', handleClick);
      document.removeEventListener('click', handleClickOutside);
      document.removeEventListener('contextmenu', handleContextMenu as any, true);
      document.removeEventListener('contextmenu', handleContentContextMenu, true);
      document.removeEventListener('contextmenu', handleContextMenu as any, false);
      document.removeEventListener('selectstart', handleSelectStart);

      if (mainContent) {
        mainContent.removeEventListener('contextmenu', handleContentContextMenu, true);
        mainContent.removeEventListener('touchstart', handleTouchStart);
        mainContent.removeEventListener('touchend', handleTouchEnd);
        mainContent.removeEventListener('touchmove', handleTouchMove);
        mainContent.removeEventListener('touchcancel', handleTouchContextMenu);
        mainContent.removeEventListener('touchforcechange', handleTouchForceChange as any);
        mainContent.removeEventListener('selectstart', handleSelectStart);
        mainContent.removeEventListener('selectionchange', handleSelectionChange);
        mainContent.removeEventListener('mousedown', handleMouseDown, true);
        mainContent.removeEventListener('mouseup', handleMouseUp, true);
        mainContent.removeEventListener('click', handleClick);
      }

      // Cleanup the style element
      cleanupStyle && cleanupStyle();
    };
  }, []);

  const handleAskClick = (e: React.MouseEvent) => {
    e.preventDefault();
    e.stopPropagation();
    if (selectedText) {
      onAskAboutSelection(selectedText);
      setShowButton(false);
    }
  };

  // Also hide the button when clicking anywhere if selection is cleared
  useEffect(() => {
    const handleGlobalClick = (e: Event) => {
      const selection = window.getSelection();
      const currentText = selection?.toString().trim();
      if (!currentText || currentText.length === 0) {
        setShowButton(false);
      }
    };

    document.addEventListener('click', handleGlobalClick as any);
    return () => {
      document.removeEventListener('click', handleGlobalClick as any);
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
      onTouchStart={(e) => e.stopPropagation()} // Prevent touch interference
      onTouchEnd={(e) => e.stopPropagation()}   // Prevent touch interference
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
