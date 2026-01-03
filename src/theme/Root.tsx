import React from 'react';
import { AuthProvider } from '../components/auth/AuthProvider';
import { ChatKitWidget } from '../components/chatkit/ChatKitWidget';

// This component wraps the entire application with the AuthProvider and ChatKit
export default function Root({ children }: { children: React.ReactNode }) {
  return (
    <AuthProvider>
      {children}
      <ChatKitWidget position="bottom-right" />
    </AuthProvider>
  );
}