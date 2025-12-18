import React, { useState, useEffect, useRef } from 'react';
import { useAuth } from '@site/src/contexts/AuthContext';
import './UserMenu.css';

export default function UserMenu() {
  const { user, signOut } = useAuth();
  const [isOpen, setIsOpen] = useState(false);
  const dropdownRef = useRef<HTMLDivElement>(null);

  // Close dropdown when clicking outside
  useEffect(() => {
    function handleClickOutside(event: MouseEvent) {
      if (dropdownRef.current && !dropdownRef.current.contains(event.target as Node)) {
        setIsOpen(false);
      }
    }

    if (isOpen) {
      document.addEventListener('mousedown', handleClickOutside);
      return () => {
        document.removeEventListener('mousedown', handleClickOutside);
      };
    }
  }, [isOpen]);

  const handleLogout = async () => {
    try {
      await signOut();
      setIsOpen(false);
      window.location.href = '/';
    } catch (error) {
      console.error('Logout failed:', error);
    }
  };

  // Not logged in: show Login + Sign Up buttons
  if (!user) {
    return (
      <div className="navbar-auth-buttons">
        <a href="/login" className="navbar-auth-link">
          Login
        </a>
        <a href="/signup" className="navbar-auth-button">
          Sign Up
        </a>
      </div>
    );
  }

  // Logged in: show user email with dropdown
  return (
    <div className="navbar-user-menu" ref={dropdownRef}>
      <button
        className="navbar-user-button"
        onClick={() => setIsOpen(!isOpen)}
        aria-expanded={isOpen}
        aria-haspopup="true"
      >
        <span className="navbar-user-email">{user.email}</span>
        <svg
          className={`navbar-user-arrow ${isOpen ? 'navbar-user-arrow--open' : ''}`}
          width="12"
          height="8"
          viewBox="0 0 12 8"
          fill="none"
          xmlns="http://www.w3.org/2000/svg"
        >
          <path
            d="M1 1.5L6 6.5L11 1.5"
            stroke="currentColor"
            strokeWidth="2"
            strokeLinecap="round"
            strokeLinejoin="round"
          />
        </svg>
      </button>

      {isOpen && (
        <div className="navbar-user-dropdown">
          <button
            className="navbar-user-dropdown-item"
            onClick={handleLogout}
          >
            Logout
          </button>
        </div>
      )}
    </div>
  );
}
