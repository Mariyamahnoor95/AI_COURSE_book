/**
 * Docusaurus Root Component (Swizzled)
 *
 * This component wraps the entire application and is rendered on every page.
 * Used to inject the ChatbotWidget globally across all pages.
 *
 * Maps to: FR-007 (Embedded chatbot on all pages)
 * Per: T033 (Swizzle Docusaurus Root)
 *
 * @see https://docusaurus.io/docs/swizzling#wrapper-your-site-with-root
 */

import React from 'react';
import ChatbotWidget from '../components/ChatbotWidget';

interface RootProps {
  children: React.ReactNode;
}

/**
 * Root wrapper component
 *
 * Renders the main application content and injects the ChatbotWidget
 * as a global floating UI element.
 *
 * The ChatbotWidget is:
 * - Fixed position (bottom-right)
 * - Available on all pages (docs, blog, custom pages)
 * - Connected to FastAPI backend (configured in T034)
 * - Session-persisted across navigation
 */
export default function Root({ children }: RootProps): JSX.Element {
  return (
    <>
      {/* Main application content from Docusaurus */}
      {children}

      {/* Global chatbot widget - rendered on all pages */}
      <ChatbotWidget />
    </>
  );
}
